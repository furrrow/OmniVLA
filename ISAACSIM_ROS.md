# OmniVLA with the DynaNav Isaac Sim benchmark

OmniVLA can run as the external ROS 2 policy for the DynaNav Nova Carter
benchmark. Isaac Sim runs from the TIC-VLA repository on the simulator machine.
OmniVLA inference, the waypoint controller, and optional SteerNav run from this
repository on the model machine.

## Pipeline

The local launcher supports two control flows:

```text
normal:
  OmniVLA -> /sim_nova/policy_path
           -> /sim_nova/waypoint -> controller -> /isaacsim/cmd_vel

steered:
  OmniVLA -> /sim_nova/policy_path -> SteerNav
           -> /sim_nova/steered_waypoint -> controller -> /isaacsim/cmd_vel
```

OmniVLA always publishes the full path. It also selects and publishes the direct
waypoint used in normal mode. The Isaac bridge does not select a policy
waypoint. Steered mode leaves the direct waypoint unused and configures the
controller to consume SteerNav's waypoint instead.

`inference/omnivla_ros.py` keeps the model loaded while DynaNav starts a fresh
Isaac Sim child for each episode. For `sim_nova`, it waits for the
transient-local `/isaacsim/episode` message and uses that message's language
instruction. A new `episode_id` clears model inputs, observations, odometry,
publication state, and temporal counters. Duplicate IDs are ignored. A
generation counter prevents inference that overlaps an episode reset from
publishing into the new episode.

Each valid prediction is converted to meters and published in `base_link`, with
positive x forward and positive y left. The path and direct waypoint come from
the same prediction. OmniVLA chooses the first point whose accumulated path arc
length reaches `lookahead_distance`; paths containing at least five points are
clamped to indices `2` through `T-3`. The `sim_nova` profile currently uses a
1.0 m lookahead.

The controller runs with `--simulation-episode-control`. It publishes zero
velocity before the first episode, clears its cached waypoint at every episode
change, waits for a waypoint from the new episode, and stops when a waypoint is
more than one second old. Its control timer uses a steady wall clock, so safety
checks continue if simulation time pauses.

## ROS topics and time

Both machines must use the same `ROS_DOMAIN_ID`. The launch scripts default to
`42`.

| Direction | Topic | Type |
| --- | --- | --- |
| Isaac to local nodes | `/clock` | `rosgraph_msgs/Clock` |
| Isaac to OmniVLA | `/isaacsim/camera/front/image_raw/compressed` | `sensor_msgs/CompressedImage` |
| Isaac to OmniVLA | `/isaacsim/odom` | `nav_msgs/Odometry` |
| Isaac to local nodes | `/isaacsim/episode` | `std_msgs/String` containing JSON |
| OmniVLA to SteerNav | `/sim_nova/policy_path` | `nav_msgs/Path` |
| OmniVLA to normal controller | `/sim_nova/waypoint` | `std_msgs/Float32MultiArray` |
| SteerNav to steered controller | `/sim_nova/steered_waypoint` | `std_msgs/Float32MultiArray` |
| Controller to Isaac | `/isaacsim/cmd_vel` | `geometry_msgs/Twist` |

The Isaac bridge publishes camera and odometry messages from one simulation
update with the same Isaac timestamp. OmniVLA preserves the camera timestamp on
the policy path. `nova_ros.bash` passes
`--ros-args -p use_sim_time:=true` to OmniVLA, the controller, and SteerNav so
all ROS nodes observe `/clock`.

Simulation timestamps can move backward when the next episode starts in a new
Isaac child. Nodes normally reset from `/isaacsim/episode`; OmniVLA also treats
a backward camera timestamp as a reset and invalidates any inference still in
progress. ROS consumers added to this pipeline should do the same or clear their
timestamped history on every new episode ID.

## Calibration

Two properties remain checkpoint-level calibration items:

- Confirm that the predicted axes are robot-frame x forward and y left.
- Confirm `metric_waypoint_spacing`. The current OmniVLA wrapper uses `0.38` m,
  while other OmniVLA checkpoints may use a different normalization scale.

The spacing directly changes the metric path and therefore which point reaches
the 1.0 m lookahead. Override it for an evaluation with
`--metric-waypoint-spacing`; do not treat static integration checks as
validation of these checkpoint-specific values.

## Prerequisites

On the OmniVLA machine:

- install the repository's `uv` environment and model checkpoints;
- install ROS 2 Humble and `tmux`;
- check out the episode-aware `custom_utils` submodule;
- configure passwordless SSH to the simulator host;
- provide a SteerNav checkout when running steered mode.

On the simulator machine, the TIC-VLA checkout must contain the current
`DynaNav`, `.env.testing`, and `run_sim.bash`. By default the scripts use host
`isaac`, repository `/home/isaac/Projects/TIC-VLA`, and X display `:1`.

## Interactive launch

Start the local normal pipeline:

```bash
cd /home/jim/Projects/OmniVLA
./nova_ros.bash normal
```

Start the SteerNav-controlled pipeline:

```bash
STEERNAV_ROOT=/home/jim/Projects/steernav ./nova_ros.bash steered
```

The controller horizon defaults to 0.75 seconds. It can be overridden for
either mode:

```bash
./nova_ros.bash normal --dt 1.0
```

To launch detached with a stable session name:

```bash
./nova_ros.bash normal --detach --session-name omnivla_manual
tmux attach -t omnivla_manual
```

The launcher writes pane output below
`logs/nova_ros/<session>/` as `inference.log`, `controller.log`, and, in steered
mode, `steernav.log`. Set `NOVA_ROS_LOG_ROOT` to move this log tree.

After inference reports `Model ready` and the controller reports
`Registered with master node`, start a simulator job from the TIC-VLA checkout:

```bash
ssh isaac
cd /home/isaac/Projects/TIC-VLA
DISPLAY=:1 ./run_sim.bash \
  --scenario DynaNav/configs/icra_scenarios_v4/scenario3_5.yaml \
  --job-id omnivla_manual_normal -- \
  --run_id omnivla_manual_normal \
  --experiment_mode normal \
  --inference_method OmniVLA
```

If the local session is stopped early, stop that exact simulator process group:

```bash
ssh isaac 'cd /home/isaac/Projects/TIC-VLA && \
  ./run_sim.bash --stop omnivla_manual_normal'
```

For a real-robot profile, run the inference node directly with its fixed
instruction:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
uv run python inference/omnivla_ros.py \
  --robot husky \
  --instruction "Move toward the blue trash bin"
```

## Batch experiments

Run normal and steered modes sequentially from outside an existing tmux client:

```bash
cd /home/jim/Projects/OmniVLA
BATCH_ID=omnivla_icra_v1 ./run_nova_experiments.bash --mode both
```

Run only one mode:

```bash
BATCH_ID=omnivla_normal_v1 \
  ./run_nova_experiments.bash --mode normal

BATCH_ID=omnivla_steered_v1 \
  ./run_nova_experiments.bash --mode steered
```

Select a scenario and controller horizon explicitly:

```bash
BATCH_ID=omnivla_scenario2_v1 \
  ./run_nova_experiments.bash \
  --mode both \
  --scenario DynaNav/configs/icra_scenarios_v4/scenario2_5.yaml \
  --dt 0.75
```

The default scenario is
`DynaNav/configs/icra_scenarios_v4/scenario3_5.yaml`. Before loading OmniVLA,
the orchestrator validates its arguments and asks the remote `run_sim.bash`
wrapper to check SSH-side paths, display access, environment, and scenario. Each
mode receives a unique job ID and matching benchmark `run_id`. Normal and
steered runs execute sequentially so they cannot share live topics. Benchmark
records use `OmniVLA` as the inference method by default; override it with
`--inference-method NAME` or the `INFERENCE_METHOD` environment variable.

The orchestrator tracks the active remote job and calls
`run_sim.bash --stop <job-id>` during signal or error cleanup. Local streamed
simulator logs are stored in `logs/orchestration/<BATCH_ID>/<mode>.log`.
Benchmark results remain under
`DynaNav/benchmark_results/<BATCH_ID>_<mode>/` in the remote TIC-VLA checkout.

Useful environment overrides are:

| Variable | Default |
| --- | --- |
| `REMOTE_HOST` | `isaac` |
| `REMOTE_REPO` | `/home/isaac/Projects/TIC-VLA` |
| `REMOTE_DISPLAY` | `:1` |
| `STEERNAV_ROOT` | `/home/jim/Projects/steernav` |
| `ROS_SETUP` | `/opt/ros/humble/setup.bash` |
| `ROS_DOMAIN_ID` | `42` |
| `INFERENCE_METHOD` | `OmniVLA` |
| `NOVA_ROS_LOG_ROOT` | `logs/nova_ros` in this repository |
| `READY_TIMEOUT_SECONDS` | `600` |
| `BATCH_ID` | Current timestamp |

## Inspect and troubleshoot

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
ros2 topic echo /isaacsim/episode std_msgs/msg/String \
  --qos-durability transient_local --once
ros2 topic echo /clock --once
ros2 topic hz /isaacsim/camera/front/image_raw/compressed
ros2 topic echo /isaacsim/odom nav_msgs/msg/Odometry --once
ros2 topic echo /sim_nova/policy_path nav_msgs/msg/Path --once
ros2 topic echo /sim_nova/waypoint std_msgs/msg/Float32MultiArray --once
ros2 topic echo /sim_nova/steered_waypoint std_msgs/msg/Float32MultiArray --once
ros2 topic echo /isaacsim/cmd_vel geometry_msgs/msg/Twist --once
```

For normal mode, confirm that each valid prediction updates both the path and
direct waypoint. For steered mode, confirm that SteerNav receives the policy
path and publishes the steered waypoint. A forward scene should produce mostly
positive path x values after axis calibration.

If readiness times out, the orchestrator prints recent output from every tmux
pane. Inspect the pane logs for model paths, CUDA or FlashAttention errors, ROS
domain mismatches, missing topics, the SteerNav checkout, SSH access, and the
remote X display. Do not run normal and steered pipelines simultaneously because
they share ROS node names and topics.
