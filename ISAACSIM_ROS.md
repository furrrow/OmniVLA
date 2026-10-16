# OmniVLA with the DynaNav Isaac Sim benchmark

This repository can run OmniVLA as the external ROS 2 policy for the DynaNav
Nova Carter benchmark. Isaac Sim runs from the TIC-VLA repository on the
simulator machine; OmniVLA inference, optional SteerNav, and the waypoint
controller run from this repository on the model machine.

## What changed

- `inference/omnivla_ros.py` now uses OmniVLA's language-only modality and takes
  each simulation instruction from the transient-local `/isaacsim/episode`
  message. Real-robot profiles continue to use `--instruction`.
- A new episode resets cached observations and inference state. Results from an
  inference that overlaps an episode change are discarded.
- Inference runs once per new camera frame after an episode, image, and odometry
  are all available. `/started` is published after the first valid path.
- Paths are validated, converted to meters, stamped from the camera observation,
  and published in `base_link` on `/sim_nova/policy_path`. The Isaac bridge
  selects an approximately 1 m lookahead waypoint.
- `inference/config/robot.yaml` now contains the `sim_nova` topics and limits.
- `inference/run_omnivla.py` is the maintained wrapper. The previous
  `run_omnivla_modified.py` name and obsolete implementation were removed, and
  ROS, video, and SCAND imports were updated.
- `nova_ros.bash` starts inference and the episode-aware waypoint controller in
  tmux, with optional SteerNav.
- `run_nova_experiments.bash` runs normal and steered experiments sequentially
  against the same remote DynaNav scenario.

The controller came from the updated `custom_utils` submodule. With
`--simulation-episode-control`, it publishes zero velocity before an episode,
clears its waypoint on each new episode ID, waits for a fresh waypoint, and
stops when waypoints become stale.

## ROS contract

Both machines must use the same `ROS_DOMAIN_ID`, which defaults to `42` in the
launch scripts.

| Direction | Topic | Type |
| --- | --- | --- |
| Isaac to OmniVLA | `/isaacsim/camera/front/image_raw/compressed` | `sensor_msgs/CompressedImage` |
| Isaac to OmniVLA | `/isaacsim/odom` | `nav_msgs/Odometry` |
| Isaac to local nodes | `/isaacsim/episode` | `std_msgs/String` containing JSON |
| OmniVLA to Isaac/SteerNav | `/sim_nova/policy_path` | `nav_msgs/Path` |
| Isaac bridge to normal controller | `/sim_nova/waypoint` | `std_msgs/Float32MultiArray` |
| SteerNav to steered controller | `/sim_nova/steered_waypoint` | `std_msgs/Float32MultiArray` |
| Controller to Isaac | `/isaacsim/cmd_vel` | `geometry_msgs/Twist` |

Paths use the robot body frame: positive x is forward, positive y is left, and
coordinates are meters. The current wrapper uses its configured
`metric_waypoint_spacing` of `0.38` m. Override it during calibration with
`--metric-waypoint-spacing`.

## Prerequisites

On the OmniVLA machine:

- install this repository's `uv` environment and model checkpoints;
- install and source ROS 2 Humble;
- install `tmux`;
- ensure the updated `custom_utils` submodule is checked out;
- configure passwordless SSH to the simulator host, normally `ssh isaac`.

On the simulator machine, the TIC-VLA checkout must contain DynaNav and its
`.env.testing` file. The default remote path is
`/home/isaac/Projects/TIC-VLA`, and the graphical benchmark expects X display
`:1`.

## Run manually

Start the local normal pipeline:

```bash
cd /home/jim/Projects/OmniVLA
./nova_ros.bash normal
```

Start the SteerNav pipeline instead:

```bash
STEERNAV_ROOT=/home/jim/Projects/steernav ./nova_ros.bash steered
```

The launcher creates a tmux session with OmniVLA inference and the waypoint
controller. Steered mode adds a third SteerNav pane. To start it detached:

```bash
./nova_ros.bash normal --detach --session-name omnivla_manual
tmux attach -t omnivla_manual
```

After local nodes report `Model ready`, start DynaNav on the simulator:

```bash
ssh isaac
cd /home/isaac/Projects/TIC-VLA
DynaNav/run_benchmark.sh \
  DynaNav/configs/icra_scenarios_v4/scenario1_5.yaml \
  --run_id omnivla_manual_scenario1
```

For a real-robot profile, run the node directly and supply a fixed instruction:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
uv run python inference/omnivla_ros.py \
  --robot husky \
  --instruction "Move toward the blue trash bin"
```

## Run the normal/steered comparison

Run both modes sequentially from outside an existing tmux client:

```bash
cd /home/jim/Projects/OmniVLA
BATCH_ID=omnivla_scenario1_v1 ./run_nova_experiments.bash
```

Select another DynaNav scenario with its path relative to the remote TIC-VLA
checkout:

```bash
BATCH_ID=omnivla_scenario2_v1 \
  ./run_nova_experiments.bash \
  DynaNav/configs/icra_scenarios_v4/scenario2_5.yaml
```

Useful environment overrides are:

| Variable | Default |
| --- | --- |
| `REMOTE_HOST` | `isaac` |
| `REMOTE_REPO` | `/home/isaac/Projects/TIC-VLA` |
| `REMOTE_DISPLAY` | `:1` |
| `STEERNAV_ROOT` | `/home/jim/Projects/steernav` |
| `ROS_SETUP` | `/opt/ros/humble/setup.bash` |
| `ROS_DOMAIN_ID` | `42` |
| `READY_TIMEOUT_SECONDS` | `600` |
| `BATCH_ID` | Current timestamp |

Remote results are written under
`DynaNav/benchmark_results/<BATCH_ID>_{normal,steered}/` in the TIC-VLA
checkout. Streamed simulator logs are saved locally under
`logs/orchestration/<BATCH_ID>/`.

## Inspect and troubleshoot

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
ros2 topic echo /isaacsim/episode std_msgs/msg/String \
  --qos-durability transient_local --once
ros2 topic hz /isaacsim/camera/front/image_raw/compressed
ros2 topic echo /sim_nova/policy_path nav_msgs/msg/Path --once
ros2 topic echo /sim_nova/waypoint std_msgs/msg/Float32MultiArray --once
```

If orchestration readiness times out, the script prints recent output from every
tmux pane. Check model paths, CUDA/FlashAttention compatibility, ROS domain and
topic visibility, the SteerNav checkout, SSH access, and the remote X display.
Do not run normal and steered pipelines simultaneously because they share ROS
node names and topics.
