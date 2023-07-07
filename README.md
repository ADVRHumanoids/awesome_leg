# Awesome Leg 

Andrea Patrizi's work on controlling our new quadruped leg prototype.

This specific branch is dedicated to the code employed for Humanoids23 submission.
Specifically, this repo holds: 
- all the code employed to perform the offline generation of the take-off trajectory and optimization of the landing configuration and joint impedances (available at `src/horizon_code/jump_generation.py` and `src/horizon_code/touchdown_opt.py`). To reproduce the solutions, you first need to install some dependencies:  [casadi with Ipopt](https://github.com/casadi/casadi), [casadi_kin_dyn](https://github.com/ADVRHumanoids/casadi_kin_dyn), [pinocchio](https://github.com/stack-of-tasks/pinocchio), [ROS Noetic](http://wiki.ros.org/noetic), [Horizon](https://github.com/ADVRHumanoids/horizon/tree/symbolic_model) (branch `symbolic_model`). Then, make sure the config files in `config/horizon/jump_generation/jump_generation.yaml` and `config/horizon/jump_generation/touchdown_opt.yaml` match the ones provided in [humanoids_opendata](https://drive.google.com/drive/folders/19J7vAJigoIES9niY9HVV40xFMkzh9XZ1).
- all the code employed to actually replay the optimizations on the real robot (at `src/xbot2_plugins/xbot2_energy_rec.py`)
- all the code to generate the plots which are in the paper (at `src/xbot2_plugins/plot_generation/`). The data used by the plot is made publicly available [here](https://drive.google.com/drive/folders/19J7vAJigoIES9niY9HVV40xFMkzh9XZ1). Just download the data and copy it in `/tmp/humanoids_opendata`. This will allow you to reproduce most of the plots. Please note that for some of them you may need to install additional dependencies like [xbot2](https://advrhumanoids.github.io/xbot2/devel/index.html).
- some additional plots are hosted in `humanoids_opendata/additional_plots` at [humanoids_opendata](https://drive.google.com/drive/folders/19J7vAJigoIES9niY9HVV40xFMkzh9XZ1).
