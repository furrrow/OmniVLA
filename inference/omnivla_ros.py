#!/usr/bin/env python3
"""ROS 2 OmniVLA language-conditioned waypoint inference.

For ``sim_nova``, instructions arrive with DynaNav episode metadata and all
observation/model state resets between episodes. Other robot profiles use the
fixed ``--instruction`` value for the lifetime of the process.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from threading import Lock
from typing import Any

import cv2
from cv_bridge import CvBridge
import matplotlib
matplotlib.use("Agg")
import numpy as np
from PIL import Image as PILImage
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Empty, Float32MultiArray, String
import torch
import yaml

from custom_utils.io_utils import load_calibration, overlay_path
from inference.run_omnivla import Inference

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = Path(__file__).resolve().parent / "config" / "robot.yaml"
DEFAULT_CAMERA_MATRIX = Path(__file__).resolve().parent / "cam_matrix.json"
DEFAULT_GOAL_IMAGE = Path(__file__).resolve().parent / "101.png"
DEFAULT_INSTRUCTION = "Move forward safely and avoid obstacles."


def load_robot_config(config_path: Path, robot: str) -> tuple[dict[str, Any], dict[str, Any]]:
    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if robot not in config:
        raise KeyError(f"Robot {robot!r} is not present in {config_path}")
    return config, config[robot]


class OmniVLANode(Node):
    def __init__(
        self,
        args: argparse.Namespace,
        config: dict[str, Any],
        robot_config: dict[str, Any],
        model: Inference,
    ) -> None:
        super().__init__("omnivla_node")
        self.args = args
        self.model = model
        self.bridge = CvBridge()
        self.lock = Lock()
        self.require_episode = args.robot == "sim_nova"
        self.episode_id: str | None = None
        self.episode_generation = 0
        self.pending_episode_reset = False
        self.instruction = args.instruction

        self.obs_img: PILImage.Image | None = None
        self.obs_header: Any | None = None
        self.obs_sequence = 0
        self.processed_sequence = 0
        self.current_pos = np.zeros(2, dtype=np.float64)
        self.current_yaw = 0.0
        self.have_odom = False
        self.started_sent = False
        self.inference_count = 0
        self.inference_start_time = time.perf_counter()

        self.rate = float(args.rate or config.get("frame_rate", 3.0))
        self.path_frame_id = args.frame_id
        self.original_img_size = (int(config["img_w"]), int(config["img_h"]))
        self.shrink_img_size = (int(config["shrink_w"]), int(config["shrink_h"]))
        self.visualize = args.visualize

        image_topic = args.image_topic or robot_config["image_topic"]
        odom_topic = args.odom_topic or robot_config["odom_topic"]
        self.compressed_img_topic = (
            args.image_type == "compressed" if args.image_type else "compressed" in image_topic
        )
        policy_path_topic = robot_config["policy_path_topic"]
        waypoint_topic = robot_config.get("waypoint_topic")
        sampled_actions_topic = robot_config["sampled_actions_topic"]
        overlay_topic = robot_config["overlay_topic"]

        self.cam_matrix = None
        self.T_cam_from_base = None
        if self.visualize:
            cam_matrix, _, T_base_from_cam = load_calibration(args.camera_matrix)
            self.cam_matrix = cam_matrix
            if T_base_from_cam is not None:
                self.T_cam_from_base = np.linalg.inv(T_base_from_cam)

        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.input_group = MutuallyExclusiveCallbackGroup()
        self.inference_group = MutuallyExclusiveCallbackGroup()

        msg_type = CompressedImage if self.compressed_img_topic else Image
        self.image_sub = self.create_subscription(
            msg_type, image_topic, self.img_callback, reliable_qos, callback_group=self.input_group
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, reliable_qos, callback_group=self.input_group
        )
        self.episode_sub = None
        if self.require_episode:
            episode_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.episode_sub = self.create_subscription(
                String,
                args.episode_topic,
                self.episode_callback,
                episode_qos,
                callback_group=self.input_group,
            )

        # In simulation, the Isaac bridge selects a lookahead point from the path.
        self.waypoint_pub = (
            None
            if self.require_episode or waypoint_topic is None
            else self.create_publisher(Float32MultiArray, waypoint_topic, reliable_qos)
        )
        self.sampled_actions_pub = self.create_publisher(
            Float32MultiArray, sampled_actions_topic, best_effort_qos
        )
        self.path_pub = self.create_publisher(PathMsg, policy_path_topic, reliable_qos)
        self.overlay_pub = self.create_publisher(Image, overlay_topic, reliable_qos)
        self.started_pub = self.create_publisher(Empty, "/started", 10)

        wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            1.0 / self.rate,
            self.run_inference_loop,
            callback_group=self.inference_group,
            clock=wall_clock,
        )

        self.get_logger().info(f"Using robot config for: {args.robot}")
        self.get_logger().info(
            f"IMAGE_TOPIC: {image_topic} compressed_img_topic: {self.compressed_img_topic}"
        )
        self.get_logger().info(f"ODOM_TOPIC: {odom_topic}")
        if self.require_episode:
            self.get_logger().info(
                f"Waiting for language instructions from {args.episode_topic}; "
                "new episodes reset OmniVLA state."
            )
            self.get_logger().info("Model ready. Waiting for episode, image, and odometry...")
        else:
            self.get_logger().info(f"Using fixed instruction: {self.instruction}")
            self.get_logger().info("Model ready. Waiting for image and odometry...")

    def episode_callback(self, msg: String) -> None:
        try:
            episode = json.loads(msg.data)
            episode_id = episode["episode_id"]
            instruction = episode["instruction"]
            if not isinstance(episode_id, str) or not episode_id:
                raise ValueError("episode_id must be a nonempty string")
            if not isinstance(instruction, str) or not instruction.strip():
                raise ValueError("instruction must be a nonempty string")
            if episode.get("state") != "started":
                return
            if episode.get("robot_type") != "nova_carter":
                raise ValueError("expected robot_type nova_carter")
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(f"Invalid episode message: {exc}")
            return

        with self.lock:
            if episode_id == self.episode_id:
                return
            self.episode_id = episode_id
            self.instruction = instruction.strip()
            self.episode_generation += 1
            self.pending_episode_reset = True
            self.obs_img = None
            self.obs_header = None
            self.obs_sequence = 0
            self.processed_sequence = 0
            self.current_pos.fill(0.0)
            self.current_yaw = 0.0
            self.have_odom = False
        self.get_logger().info(f"Started episode {episode_id}; instruction: {self.instruction}")

    def img_callback(self, msg: Image | CompressedImage) -> None:
        try:
            if self.compressed_img_topic:
                frame_bgr = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:
                frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            image = PILImage.fromarray(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
            if image.size != self.shrink_img_size:
                image = image.resize(self.shrink_img_size)
            with self.lock:
                self.obs_img = image
                self.obs_header = msg.header
                self.obs_sequence += 1
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")

    def odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = R.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")[2]
        with self.lock:
            self.current_pos[:] = [p.x, p.y]
            self.current_yaw = float(yaw)
            self.have_odom = True

    def reset_episode_state(self) -> None:
        self.model.count_id = 0
        self.model.waypoints = None
        self.started_sent = False
        self.inference_count = 0
        self.inference_start_time = time.perf_counter()

    def to_path_msg(self, path_xy: np.ndarray, stamp: Any) -> PathMsg:
        msg = PathMsg()
        msg.header.stamp = stamp
        msg.header.frame_id = self.path_frame_id
        for x, y in path_xy:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def run_inference_loop(self) -> None:
        with self.lock:
            reset_requested = self.pending_episode_reset
            reset_generation = self.episode_generation
            self.pending_episode_reset = False
        if reset_requested:
            self.reset_episode_state()

        with self.lock:
            if self.pending_episode_reset or self.episode_generation != reset_generation:
                return
            if self.require_episode and self.episode_id is None:
                return
            if self.obs_img is None or self.obs_header is None:
                return
            if not self.have_odom:
                self.get_logger().info("waiting on odom", throttle_duration_sec=2.0)
                return
            if self.obs_sequence == self.processed_sequence:
                return
            image = self.obs_img.copy()
            header = self.obs_header
            sequence = self.obs_sequence
            current_pos = self.current_pos.copy()
            current_yaw = self.current_yaw
            instruction = self.instruction
            generation = self.episode_generation
            self.processed_sequence = sequence

        # These are masked in language-only mode, but preprocessing still needs placeholders.
        self.model.update_current_state(image, current_pos, current_yaw)
        self.model.update_goal(
            goal_image_PIL=self.args.goal_image,
            goal_utm=current_pos,
            goal_compass=current_yaw,
            lan_inst_prompt=instruction,
        )
        started = time.perf_counter()
        with torch.inference_mode():
            self.model.run()
        inference_seconds = time.perf_counter() - started

        waypoints = np.asarray(self.model.waypoints)
        if waypoints.ndim == 3 and waypoints.shape[0] == 1:
            waypoints = waypoints[0]
        if waypoints.ndim != 2 or waypoints.shape[1] < 2 or len(waypoints) == 0:
            raise ValueError(f"Expected OmniVLA waypoints shaped (T,2+), got {waypoints.shape}")
        path_xy = waypoints[:, :2].astype(np.float64) * float(self.model.metric_waypoint_spacing)
        if not np.isfinite(path_xy).all():
            raise ValueError("OmniVLA returned non-finite waypoints")

        with self.lock:
            if generation != self.episode_generation:
                return
            self.path_pub.publish(self.to_path_msg(path_xy, header.stamp))
            if self.waypoint_pub is not None:
                index = min(max(self.args.waypoint_idx, 0), len(path_xy) - 1)
                waypoint = Float32MultiArray()
                waypoint.data = path_xy[index].astype(float).tolist()
                self.waypoint_pub.publish(waypoint)
            sampled = Float32MultiArray()
            sampled.data = path_xy.flatten().astype(float).tolist()
            self.sampled_actions_pub.publish(sampled)
            if not self.started_sent:
                self.started_sent = True
                self.started_pub.publish(Empty())
                self.get_logger().info("Published /started after first valid path.")

        if self.visualize and self.cam_matrix is not None and self.T_cam_from_base is not None:
            overlay = overlay_path(
                trajectories=path_xy,
                img=np.asarray(image.resize(self.original_img_size)),
                cam_matrix=self.cam_matrix,
                T_cam_from_base=self.T_cam_from_base,
            )
            overlay_msg = self.bridge.cv2_to_imgmsg(np.asarray(overlay), encoding="rgb8")
            overlay_msg.header = header
            with self.lock:
                if generation == self.episode_generation:
                    self.overlay_pub.publish(overlay_msg)

        self.inference_count += 1
        elapsed = time.perf_counter() - self.inference_start_time
        if elapsed >= 1.0:
            self.get_logger().info(
                f"Inference rate: {self.inference_count / elapsed:.2f} Hz | "
                f"last={inference_seconds:.2f}s path_points={len(path_xy)}"
            )
            self.inference_count = 0
            self.inference_start_time = time.perf_counter()


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-r", "--robot", default="husky", help="Robot key in the YAML config")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--image-topic")
    parser.add_argument("--image-type", choices=("raw", "compressed"))
    parser.add_argument("--odom-topic")
    parser.add_argument("--episode-topic", default="/isaacsim/episode")
    parser.add_argument("-i", "--instruction", default=DEFAULT_INSTRUCTION)
    parser.add_argument("--rate", type=float)
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--waypoint-idx", type=int, default=4)
    parser.add_argument("--camera-matrix", default=str(DEFAULT_CAMERA_MATRIX))
    parser.add_argument("--goal-image", default=str(DEFAULT_GOAL_IMAGE))
    parser.add_argument("--metric-waypoint-spacing", type=float, default=None)
    parser.add_argument("--visualize", action="store_true")
    args, ros_args = parser.parse_known_args()

    if ros_args and ros_args[0] != "--ros-args":
        parser.error(f"Unrecognized arguments: {ros_args}")
    if args.rate is not None and (not math.isfinite(args.rate) or args.rate <= 0):
        parser.error("--rate must be finite and greater than zero")
    if args.metric_waypoint_spacing is not None and (
        not math.isfinite(args.metric_waypoint_spacing) or args.metric_waypoint_spacing <= 0
    ):
        parser.error("--metric-waypoint-spacing must be finite and greater than zero")
    if not args.instruction.strip():
        parser.error("--instruction must be nonempty")
    args.goal_image = PILImage.open(Path(args.goal_image).expanduser()).convert("RGB")
    return args, ros_args


def main() -> int:
    args, ros_args = parse_args()
    config, robot_config = load_robot_config(Path(args.config).expanduser().resolve(), args.robot)
    modality = {
        "pose_goal": False,
        "satellite": False,
        "image_goal": False,
        "lan_prompt": True,
    }
    model = Inference(
        save_dir=str(REPO_ROOT / "inference"),
        ego_frame_mode=True,
        modality=modality,
        save_images=False,
        radians=True,
    )
    if args.metric_waypoint_spacing is not None:
        model.metric_waypoint_spacing = args.metric_waypoint_spacing

    rclpy.init(args=ros_args)
    node = OmniVLANode(args, config, robot_config, model)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
