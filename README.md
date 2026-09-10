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
!! Using [inference/run_omnivla_modified.py](inference/run_omnivla_modified.py) as base, please diff and check out the changes compared to 
the original [inference/run_omnivla.py](inference/run_omnivla.py)

#### 1 ./inference/video_inference.py does continuous inference over a video
- uses images inside the inference/ as start and goal images

#### 2./inference/omnivla_ros.py does inference while publishing ros messages, and a visual overlay
- its configs are derived from [/inference/config/robot.yaml](/inference/config/robot.yaml)
- requires both camera and odometry to run
- path overlay are published as "/{robot_name}/path_overlay" unless otherwise specified
