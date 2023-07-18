### <center>Awesome Leg repository</center> 

This specific branch hosts the code employed for Humanoids23 submission of the paper "_Optimal Design of Agile Jumping Maneuvers for a Single Leg System_".
 

##### <center> Take-off </center>
|optimization|simulation|hardware|   
|:----------------------------------------------------------------------------:|:----------------------------------------------------------------------------------------------------------------:|:----------------------------------------------------------------------------------------------------------------:|
|  ![takeoff_opt](docs/animations/takeoff_opt.gif)| ![takeoff_replay_gazebo](docs/animations/takeoff_replay_gazebo.gif) | ![takeoff_replay_hardware](docs/animations/takeoff_replay_hardware.gif) 	

##### <center> Touchdown </center>

|optimization|simulation|hardware|   
|:----------------------------------------------------------------------------:|:----------------------------------------------------------------------------------------------------------------:|:----------------------------------------------------------------------------------------------------------------:|
|  ![touchdown_opt](docs/animations/touchdown_opt.gif)| ![opt_landing_gazebo](docs/animations/opt_landing_gazebo.gif) | ![takeoff_replay_hardware](docs/animations/opt_landing_hardware.gif) 	

##### What can be found here?
- An open access link to all the employed data: [humanoids_opendata](https://drive.google.com/drive/folders/19J7vAJigoIES9niY9HVV40xFMkzh9XZ1).
- A ready-to-use way of reproducing all the results and plots using the provided [docker_image](https://hub.docker.com/repository/docker/andpatr/awesome_leg_humanoids23/general):
  - Install [Docker](https://docs.docker.com/engine/install/ubuntu/) on you system and create an account if you do not possess one.
  - clone this repo on you host machine and source `scripts/docker_setup.bash`. 
  - ensure you have at least 50GB of free space on your system and pull the docker image with `docker pull andpatr/awesome_leg_humanoids23`. This may take a while depending on your internet connection.
  - run the docker with `run-container ${dummy_name} andpatr/awesome_leg_humanoids23`. This will open up a terminal within the container (note both the username and password are "user"). You will find some synthetic instructions on how to play with it at `docs/docker/README_humanoids23.md`.
- All the code employed to perform the offline generation of the take-off trajectory and optimization of the landing configuration and joint impedances. Specifically, the code for the take-off and touchdown can be found, respectively, at `src/horizon_code/jump_generation.py` and `src/horizon_code/touchdown_opt.py`. 
- All the code employed to actually replay the optimizations in simulation and on the real robot (at `src/xbot2_plugins/xbot2_energy_rec/`). The code is based on the combination of the real-time middleware for robotics [xbot2](https://advrhumanoids.github.io/xbot2/devel/index.html) and the [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP/tree/v3.8) library. Since XBot2 is currently not open-sourced, to be able to run the simulations you will need to use the provided [docker_image](https://hub.docker.com/repository/docker/andpatr/awesome_leg_humanoids23/general), with precompiled versions of all the necessary libraries.
- Additionally, the code employed to generate the plots in the paper (at `src/xbot2_plugins/plot_generation/`). To reproduce the plots, please follow the instructions at `docs/docker/README_humanoids23.md`.