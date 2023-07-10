#!/usr/bin/env python3

import argparse

import rospkg

# from jump_utils.energy_rec import landingEnergyRecover
from jump_utils.energy_rec import landingEnergyRecover

import subprocess

from termcolor import colored

import os

import shutil

from datetime import datetime
from datetime import date

from jump_utils.miscell_utils import str2bool

file_name = os.path.splitext(os.path.basename(__file__))[0]
file_name.replace(".py", "")

unique_id = date.today().strftime("%d-%m-%Y") + "-" + \
            datetime.now().strftime("%H_%M_%S")

sol_mat_basename = "awesome_jump"

def main(args):
    
    landing_generator_ig = landingEnergyRecover(horizon_config_fullpath,
                                             actuators_config_fullpath,
                                             urdf_full_path,
                                             args.results_dir, 
                                             is_ref_prb=False)

    # we first solve for the initial guess (potentially coarser problem)
    landing_generator_ig.init_prb()
    landing_generator_ig.setup_prb()
    landing_generator_ig.solve_prb()
    landing_generator_ig.postproc_sol()

    # landing_generator = landingEnergyRecover(horizon_config_fullpath,
    #                                         actuators_config_fullpath,
    #                                         urdf_full_path,
    #                                         args.results_dir, 
    #                                         is_ref_prb=True)
    # # and then for the final solution

    # landing_generator.init_prb()
    # landing_generator.setup_prb()
    # landing_generator.solve_prb()
    # landing_generator.postproc_sol()

if __name__ == '__main__':

    parser = argparse.ArgumentParser(
        description='Pipeline script for the generation of offline jumps of the Awesome Leg')

    # first level specific arguments
    parser.add_argument('--urdf_path', '-urdf', type=str, default=rospkg.RosPack().get_path(
        "awesome_leg") + "/description/urdf" + "/" + "awesome_leg_test_rig" + ".urdf")

    parser.add_argument('-sol_mat_name', "-matn", type=str, default=sol_mat_basename)

    parser.add_argument('--results_dir', '-rdir', type=str, help='where results are saved',
                        default="/tmp/" + file_name + "_" + unique_id) #

    parser.add_argument('--hor_confname', '-hconf', type=str,
                        help='horizon config file name', default=file_name)

    args = parser.parse_args()

    rospackage = rospkg.RosPack()  # Only for taking the path to the leg package
    exec_path = rospackage.get_path("awesome_leg") + "/src/horizon_code"
    urdfs_path = rospackage.get_path("awesome_leg") + "/description/urdf"
    urdf_name = "awesome_leg_test_rig"
    urdf_full_path = urdfs_path + "/" + urdf_name + ".urdf"
    xacro_full_path = urdfs_path + "/" + urdf_name + ".urdf.xacro"

    sliding_guide_command = "sliding_guide:=" + "true"

    config_path = rospackage.get_path("awesome_leg") + "/config/"  # configuration files path
    horizon_config_path = config_path + "horizon/touchdown_opt/"
    horizon_config_fullpath = horizon_config_path + args.hor_confname + ".yaml"
    actuators_config_fullpath = config_path + "actuators.yaml"

    try:

        print(colored("\n--> GENERATING LEG URDF...\n", "blue"))
        xacro_gen = subprocess.check_call(["xacro",
                                           xacro_full_path,
                                           sliding_guide_command,
                                           "-o",
                                           urdf_full_path])

        print(colored("\n--> URDF GENERATED SUCCESSFULLY. \n", "blue"))

    except:

        print(colored('FAILED TO GENERATE URDF.', "red"))

    if not os.path.isdir(args.results_dir):
        os.mkdir(args.results_dir)
    shutil.copyfile(horizon_config_fullpath, args.results_dir + "/" + "touchdown_opt.yaml")
    shutil.copyfile(actuators_config_fullpath, args.results_dir + "/actuators" + unique_id + ".yaml")
    shutil.copyfile(xacro_full_path, args.results_dir + "/" + urdf_name + ".urdf.xacro")
    shutil.copyfile(urdf_full_path, args.results_dir + "/" + urdf_name + ".urdf")

    main(args)
