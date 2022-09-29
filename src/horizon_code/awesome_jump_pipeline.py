#!/usr/bin/env python3
import os, argparse

import rospkg

from datetime import datetime
from datetime import date

from termcolor import colored

from jump_utils.miscell_utils import str2bool

import shutil

import subprocess 

file_name = os.path.splitext(os.path.basename(__file__))[0]
file_name.replace(".py", "")

unique_id = date.today().strftime("%d-%m-%Y") + "-" +\
                    datetime.now().strftime("%H_%M_%S")

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(
        description='Pipeline script for the generation of offline jumps of the Awesome Leg')

    parser.add_argument('--results_dir', '-rdir', type = str,\
                        help = 'where results are saved', default = "/tmp/" + file_name + "_" + unique_id)

    parser.add_argument('--hor_confname', '-hconf', type = str,\
                        help = 'horizon config file name', default = file_name)
    
    parser.add_argument('--sol_mat_name', '-matn', type = str,\
                        help = 'horizon config file name', default = "awesome_jump")
    
    parser.add_argument('--sol_mat_name_res', '-matn_res', type = str,\
                        help = 'horizon config file name', default = "awesome_jump_res")

    parser.add_argument('--awesome_jump_ref', '-matn_ref', type = str,\
                        help = 'horizon config file name', default = "awesome_jump_ref")

    args = parser.parse_args()

    rospackage = rospkg.RosPack() # Only for taking the path to the leg package
    exec_path = rospackage.get_path("awesome_leg") + "/src/horizon_code"
    urdfs_path = rospackage.get_path("awesome_leg") + "/description/urdf"
    urdf_name = "awesome_leg_test_rig"
    urdf_full_path = urdfs_path + "/" + urdf_name + ".urdf"
    xacro_full_path = urdfs_path + "/" + urdf_name + ".urdf.xacro"

    config_path=rospackage.get_path("awesome_leg")+"/config/" # configuration files path
    horizon_config_path = config_path + "horizon/"
    horizon_config_fullpath = horizon_config_path + args.hor_confname + ".yaml"
    actuators_config_fullpath = config_path + "actuators.yaml"

    os.mkdir(args.results_dir)
    shutil.copyfile(actuators_config_fullpath, args.results_dir + "/actuators_" + unique_id + ".yaml") 
    shutil.copyfile(horizon_config_fullpath, args.results_dir + "/" + "horizon_config.yaml" )
    shutil.copyfile(xacro_full_path, args.results_dir + "/" + urdf_name + ".urdf.xacro" )
    shutil.copyfile(urdf_full_path, args.results_dir + "/" + urdf_name + ".urdf" )

    sliding_guide_command = "sliding_guide:=" + "true"

    try:

        print(colored("\n--> GENERATING LEG URDF...\n", "blue"))
        xacro_gen = subprocess.check_call(["xacro",\
                                        xacro_full_path, \
                                        sliding_guide_command, \
                                        "-o", 
                                        urdf_full_path])

        print(colored("\n--> URDF GENERATED SUCCESSFULLY. \n", "blue"))

    except:

        print(colored('FAILED TO GENERATE URDF.', "red"))

    try:

        rviz_window = subprocess.Popen(["roslaunch",\
                                        "awesome_leg",\
                                        "horizon_rviz.launch", \
                                        sliding_guide_command])

    except:

        print('Failed to launch RViz.')

    os.chdir(exec_path) # change current path, so that executable can be run with check_call

    # try:

    print(colored("\n--> STARTING GENERATION OF RESAMPLED JUMP TRAJ. ....\n", "blue"))
    reset_term = subprocess.check_call(["reset"])

    subprocess.check_call(["./awesome_jump_res_gen.py", 
                            "-urdf", urdf_full_path, 
                            "-id", unique_id, 
                            "-yaml", horizon_config_fullpath, 
                            "-ayaml", actuators_config_fullpath,
                            "-matn", args.sol_mat_name, 
                            "-matn_res", args.sol_mat_name_res, 
                            "-rdir", args.results_dir])

    print(colored("\n--> GENERATION OF RESAMPLED JUMP TRAJ. FINISHED SUCCESSFULLY. \n", "blue"))

    # except:

    #     print(colored('\n An exception occurred while running the generation . Muy malo!!! \n', "red"))


    # try:

    # print(colored("\n--> STARTING SECOND LEVEL OPTIMIZATION....\n", "blue"))

    # #run first level (blocking --> we have to wait for data to be dumped to file)
    # second_level_proc = subprocess.check_call(["./run_2nd_3rd_step_opt_on_workstation.py", \
    #                             "-d", \
    #                             res_dir_full_name, \
    #                             "-dfn", \
    #                             l2_dump_folder_name,
    #                             "-rdbs", \
    #                             res_dir_basename, \
    #                             "-ldn", \
    #                             l1_dump_folder_name, \
    #                             "-ipopt_v", \
    #                             str(args.ipopt_verb_lev), \
    #                             "-nc",\
    #                             str(args.n_clust_l2), 
    #                             "-ma57", \
    #                             str(args.use_ma57), 
    #                             "-igs", \
    #                             str(args.ig_seed_l2),
    #                             "-mtf", \
    #                             str(args.max_trials_factor_l2), \
    #                             "-mst",
    #                             str(args.multistart_n_l2), \
    #                             "-urdf", \
    #                             urdf_full_path, \
    #                             "-coll", \
    #                             coll_yaml_path])

    #     print(colored("\n--> SECOND LEVEL OPTIMIZATION FINISHED SUCCESSFULLY. \n", "blue"))

    # except:

    #     print(colored('\n An exception occurred while running the second level of the codesign pipeline. Muy malo!!! \n', "red"))
    
