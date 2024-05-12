# CoppeliaUR10_BiRRT_GM_GB
Experiments with BiRRT algorithm using Gaussian Mixture trained on collected data with GoalBias on UR10 manipulator from CoppeliaSim.


# Installation

### Clone this repository
```bash
git clone git@github.com:euskov17/CoppeliaUR10_BiRRT_GM_GB.git
cd CoppeliaUR10_BiRRT_GM_GB
```

### Install requirements

```bash
python install -r requirements.txt
```

### Install CoppeliaSim

Install CoppeliaSim application from https://coppeliarobotics.com/

# Running examples

Example of running Bi-RRT algorithm is presented in [example.ipynb](./example.ipynb).

Before running example run `./coppeliaSim` and load scene in application `file->load_scene`.

Using for experiments scene is located in [scenes/ur10_scene_red.ttt](./scenes/ur10_scene.ttt)

![alt text](images/scene.png "Title")


# Visualisation examples
<!-- !["Visualisation of path made by BiRRT with uniform sampler"](https://github.com/euskov17/CoppeliaUR10_BiRRT_GM_GB/videos/uniform.mp4)

!["Visualisation of path made by BiRRT with GM sampler"](https://github.com/euskov17/CoppeliaUR10_BiRRT_GM_GB/videos/uniform.mp4)

!["Visualisation of path made by BiRRT with GM sampler with goal bias"](https://github.com/euskov17/CoppeliaUR10_BiRRT_GM_GB/videos/uniform.mp4) -->


<video width="320" height="240" controls>
  <source src="videos/uniform.mp4" type="video/mp4">
</video>


<video width="320" height="240" controls>
  <source src="videos/gm_sampler.mp4" type="video/mp4">
</video>


<video width="320" height="240" controls>
  <source src="videos/gm_gb_sampler.mp4" type="video/mp4">
</video>