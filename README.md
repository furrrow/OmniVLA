# OmniVLA: custom branch
This is a custom branch to get omnivla running with robots. 
Differences from the original repo will be outlined here

### Installation
Opting for python 3.10
UV package manager is preferred, it references the updated [pyproject.toml](pyproject.toml)
```
uv sync   
```

remember to get the weights:
```
git clone https://huggingface.co/NHirose/omnivla-original
git clone https://huggingface.co/NHirose/omnivla-original-balance    
git clone https://huggingface.co/NHirose/omnivla-finetuned-cast 
```
### Inference
The maintained inference wrapper is [inference/run_omnivla.py](inference/run_omnivla.py).

For language-conditioned evaluation with the DynaNav Isaac Sim benchmark, see
[ISAACSIM_ROS.md](ISAACSIM_ROS.md).

### Multi-scenario Nova experiments

Run all three five-episode scenarios sequentially with:

```bash
./run_nova_scenarios.bash scenario1_5 scenario2_5 scenario3_5
```

The default mode is `ablation`, so every scenario runs `normal`, `steered`, and
`no-velocity`. Pass `--mode` to select a different mode. The wrapper uses one
timestamp as its batch prefix and adds the scenario name. For example, a prefix
of `20261018_193500` creates:

```text
logs/orchestration/20261018_193500_scenario1_5/
logs/orchestration/20261018_193500_scenario2_5/
logs/orchestration/20261018_193500_scenario3_5/
```

The corresponding remote result directories add the mode, for example:

```text
DynaNav/benchmark_results/20261018_193500_scenario1_5_normal/
DynaNav/benchmark_results/20261018_193500_scenario1_5_steered/
DynaNav/benchmark_results/20261018_193500_scenario1_5_no-velocity/
```

Set `BATCH_ID` to choose the shared prefix:

```bash
BATCH_ID=omnivla_eval ./run_nova_scenarios.bash \
  scenario1_5 scenario2_5 scenario3_5
```

#### 1 ./inference/video_inference.py does continuous inference over a video
- uses images inside the inference/ as start and goal images

#### 2./inference/omnivla_ros.py does inference while publishing ros messages, and a visual overlay
- its configs are derived from [/inference/config/robot.yaml](/inference/config/robot.yaml)
- requires both camera and odometry to run
- path overlay are published as "/{robot_name}/path_overlay" unless otherwise specified
