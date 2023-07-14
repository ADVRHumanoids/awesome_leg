### <center>Awesome Leg repository</center> 

This specific branch hosts the code employed for Humanoids23 submission of the paper "_Optimal Design of Agile Jumping Maneuvers for a Single Leg System_".
 

##### <center> Take-off </center>
|optimization|simulation|hardware|   
|:----------------------------------------------------------------------------:|:----------------------------------------------------------------------------------------------------------------:|:----------------------------------------------------------------------------------------------------------------:|
|  ![takeoff_opt](docs/animations/takeoff_opt.gif)| ![takeoff_replay_gazebo](docs/animations/takeoff_replay_gazebo.gif) | ![takeoff_replay_hardware](docs/animations/takeoff_replay_hardware.gif) 	

##### <center> Touchdown </center>

|optimization|simulation|hardware|   
|:----------------------------------------------------------------------------:|:----------------------------------------------------------------------------------------------------------------:|:----------------------------------------------------------------------------------------------------------------:|
|  ![touchdown_opt](docs/animations/touchdown_opt.gif)| ![opt_landing_gazebo](docs/animations/opt_landing_gazebo.gif) | ![takeoff_replay_hardware](docs/animations/takeoff_replay_hardware.gif) 	

##### What can be found here?
- an open access link to all the employed data: [humanoids_opendata](https://drive.google.com/drive/folders/19J7vAJigoIES9niY9HVV40xFMkzh9XZ1).
- all the code employed to perform the offline generation of the take-off trajectory and optimization of the landing configuration and joint impedances. 
Specifically, the code for the take-off and touchdown can be found, respectively, at `src/horizon_code/jump_generation.py` and `src/horizon_code/touchdown_opt.py`. 
To reproduce the solutions
    - first, you need to install some dependencies:  
        - [casadi with Ipopt](https://github.com/casadi/casadi)
        - [casadi_kin_dyn](https://github.com/ADVRHumanoids/casadi_kin_dyn)
        - [pinocchio](https://github.com/stack-of-tasks/pinocchio)
        - [ROS Noetic](http://wiki.ros.org/noetic)
        - [Horizon](https://github.com/ADVRHumanoids/horizon/tree/symbolic_model) (branch `f_opt`)
    - then modify the `CMakeLists.txt` file and remove the unnecessary packages dependencies and the compilation of the control plugins in `src/xbot2_plugins`. Compile and install these package, so that URDF description files are available at the ROS level.
    - then, make sure the config files in `config/horizon/jump_generation/jump_generation.yaml` and `config/horizon/jump_generation/touchdown_opt.yaml` match the ones provided in [humanoids_opendata](https://drive.google.com/drive/folders/19J7vAJigoIES9niY9HVV40xFMkzh9XZ1), folders `opt_jumps/jump_generation_06-07-2023-14_24_01/horizon_config.yaml` and `opt_landings/touchdown_opt_11-07-2023-12_25_45/touchdown_opt.yaml`

    - at this point you should be able to run the take-off optimization with `python3 jump_generation.py` and the touchdown optimization with `python3 touchdown_opt.py`. Each of these scripts will dump the solutions under the folders `/tmp/touchdown_opt_${date}-${time}` and `/tmp/jump_generation_${date}-${time}`, respectively.


- all the code employed to actually replay the optimizations in simulation and on the real robot (at `src/xbot2_plugins/xbot2_energy_rec/`). The code is based on the combination of the real-time middleware for robotics [xbot2](https://advrhumanoids.github.io/xbot2/devel/index.html) and the [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP/tree/v3.8) library. Since XBot2 is currently not open-sourced, to be able to run the simulations you will need to use the provided [docker_image](), with precompiled versions of all the necessary libraries.
- additionally, all the code employed to generate the plots in the paper (at `src/xbot2_plugins/plot_generation/`). The data used by the plot is made publicly available [humanoids_opendata](https://drive.google.com/drive/folders/19J7vAJigoIES9niY9HVV40xFMkzh9XZ1). Just download the data and copy it in `/tmp/humanoids_opendata`. This will allow you to reproduce most of the plots, provided that you first install all the required dependencies.