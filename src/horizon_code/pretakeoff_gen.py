#!/usr/bin/env python3

import argparse

import rospkg

from jump_utils.jump_tasks import preTakeoffTO

import subprocess

from termcolor import colored

import os

import shutil

from datetime import datetime
from datetime import date

file_name = os.path.splitext(os.path.basename(__file__))[0]
file_name.replace(".py", "")

unique_id = date.today().strftime("%d-%m-%Y") + "-" +\
                    datetime.now().strftime("%H_%M_%S")

def main(args):

    jump_generator = preTakeoffTO(horizon_config_fullpath, 
                    actuators_config_fullpath, 
                    urdf_full_path,
                    args.results_dir, 
                    sol_mat_name = args.sol_mat_name, 
                    sol_mat_name_res = args.sol_mat_name_res)

    jump_generator.init_prb()
    jump_generator.setup_prb()

    jump_generator.solve_prb()

    jump_generator.postproc_sol()

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(
        description='Pipeline script for the generation of offline jumps of the Awesome Leg')

    # first level specific arguments
    parser.add_argument('--urdf_path', '-urdf', type = str, default= rospkg.RosPack().get_path("awesome_leg") + "/description/urdf"+ "/" + "awesome_leg_test_rig" + ".urdf")
    parser.add_argument('-sol_mat_name', "-matn", type = str, default = "awesome_jump")
    parser.add_argument('-sol_mat_name_res', "-matn_res", type = str, default = "awesome_jump_res")
    parser.add_argument('--results_dir', '-rdir', type = str, help = 'where results are saved', default = "/tmp/" + file_name + "_" + unique_id)
    parser.add_argument('--hor_confname', '-hconf', type = str,\
                        help = 'horizon config file name', default = file_name)

    args = parser.parse_args()

    rospackage = rospkg.RosPack() # Only for taking the path to the leg package
    exec_path = rospackage.get_path("awesome_leg") + "/src/horizon_code"
    urdfs_path = rospackage.get_path("awesome_leg") + "/description/urdf"
    urdf_name = "awesome_leg_test_rig"
    urdf_full_path = urdfs_path + "/" + urdf_name + ".urdf"
    xacro_full_path = urdfs_path + "/" + urdf_name + ".urdf.xacro"
    
    sliding_guide_command = "sliding_guide:=" + "true"

    config_path=rospackage.get_path("awesome_leg")+"/config/" # configuration files path
    horizon_config_path = config_path + "horizon/jump_controller/"
    horizon_config_fullpath = horizon_config_path + args.hor_confname + ".yaml"
    actuators_config_fullpath = config_path + "actuators.yaml"

    os.mkdir(args.results_dir)
    shutil.copyfile(actuators_config_fullpath, args.results_dir + "/actuators" + unique_id + ".yaml") 
    shutil.copyfile(horizon_config_fullpath, args.results_dir + "/" + "horizon_config.yaml" )
    shutil.copyfile(xacro_full_path, args.results_dir + "/" + urdf_name + ".urdf.xacro" )
    shutil.copyfile(urdf_full_path, args.results_dir + "/" + urdf_name + ".urdf" )

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

    main(args)