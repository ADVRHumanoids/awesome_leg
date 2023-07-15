#### Brief instructions on how to play with this docker:

- The code for the take-off and touchdown generation can be found, respectively, at `~/andreap_ws/src/awesome_leg/src/horizon_code/jump_generation.py` and `~/andreap_ws/src/awesome_leg/src/horizon_code/touchdown_opt.py`. 
  
   To reproduce the optimal solutions
    - make sure the config files in `~/andreap_ws/src/awesome_leg/config/horizon/jump_generation/jump_generation.yaml` and `awesome_leg/config/horizon/jump_generation/touchdown_opt.yaml` match the ones provided at  `/tmp/humanoids_opendata/opt_jumps/jump_generation_06-07-2023-14_24_01/horizon_config.yaml` and `/tmphumanoids_opendata/opt_landings/touchdown_opt_11-07-2023-12_25_45/touchdown_opt.yaml` (they should match by default).
    - at this point you should be able to run the take-off optimization with `python3 ~/andreap_ws/src/awesome_leg/src/horizon_code/jump_generation.py` and the touchdown optimization with `python3 ~/andreap_ws/src/awesome_leg/src/horizon_code/touchdown_opt.py`. Each of these scripts will dump the solutions in .mat files under the folders `/tmp/touchdown_opt_${date}-${time}` and `/tmp/jump_generation_${date}-${time}`, respectively. Please note you might experience slightly different convergence performance due to the absence of the MA57 library in the docker (MUMPS is used instead).

    To replay the optimal solutions run `~/andreap_ws/src/awesome_leg/src/rviz_traj_replayer.py -path /tmp/humanoids_opendata/opt_jumps/jump_generation_06-07-2023-14_24_01 -fname apex_awesome_jump_ref` for the take-off and `~/andreap_ws/src/awesome_leg/src/rviz_traj_replayer.py -path  /tmp/humanoids_opendata/opt_landings/touchdown_opt_11-07-2023-12_25_45 -fname energy_recov_ig` for the touchdown, respectively.
- To spawn a Gazebo simulation with the leg run `roslaunch awesome_leg gazebo.launch`. After that, the steps to replay the takeoff + touchdown are the following: 
	
