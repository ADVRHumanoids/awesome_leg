#!/usr/bin/env python3

import argparse

import rospkg

# from jump_utils.pretakeoff import preTakeoffTO
from jump_utils.up2apex import up2ApexGen
# from jump_utils.full_gen import fullJumpGen

import subprocess

from termcolor import colored

import os

import shutil

from datetime import datetime
from datetime import date

from jump_utils.miscell_utils import str2bool

file_name = os.path.splitext(os.path.basename(__file__))[0]
file_name.replace(".py", "")

unique_id = date.today().strftime("%d-%m-%Y") + "-" +\
                    datetime.now().strftime("%H_%M_%S")

pretakeoff_opt_name = "pretakeoff"
apex_opt_name = "apex"
full_opt_name = "full"

sol_mat_basename = "awesome_jump"

def main(args):

    # if args.opt_type == pretakeoff_opt_name:

    #     jump_generator = preTakeoffTO(horizon_config_fullpath, 
    #                 actuators_config_fullpath, 
    #                 urdf_full_path,
    #                 args.results_dir, 
    #                 sol_mat_name = pretakeoff_opt_name + "_" + args.sol_mat_name, 
    #                 sol_mat_name_res = pretakeoff_opt_name + "_" + args.sol_mat_name_res, 
    #                 acc_based_formulation = args.is_acc_based_form)

    #     jump_generator.init_prb()

    #     jump_generator.setup_prb()

    #     jump_generator.solve_prb()

    #     jump_generator.postproc_sol()

    if args.opt_type == apex_opt_name:
        
        # raw + resampled solution generation
        jump_generator = up2ApexGen(horizon_config_fullpath, 
                    actuators_config_fullpath, 
                    urdf_full_path,
                    args.results_dir, 
                    sol_mat_name = apex_opt_name + "_" + args.sol_mat_name, 
                    sol_mat_name_res = apex_opt_name + "_" + args.sol_mat_name_res, 
                    sol_mat_name_ref = apex_opt_name + "_" + args.sol_mat_name_ref,
                    is_ref_prb = False, 
                    acc_based_formulation = args.is_acc_based_form)

        jump_generator.init_prb()
        
        jump_generator.setup_prb()

        jump_generator.solve_prb()

        jump_generator.postproc_sol()

        if args.refine:
            # refined solution generation
            jump_generator = up2ApexGen(horizon_config_fullpath, 
                        actuators_config_fullpath, 
                        urdf_full_path,
                        args.results_dir, 
                        sol_mat_name = apex_opt_name + "_" + args.sol_mat_name, 
                        sol_mat_name_res = apex_opt_name + "_" + args.sol_mat_name_res, 
                        sol_mat_name_ref = apex_opt_name + "_" + args.sol_mat_name_ref,
                        is_ref_prb = True, 
                        load_ig = args.load_ig, 
                        acc_based_formulation = args.is_acc_based_form)

            jump_generator.init_prb()
            
            jump_generator.setup_prb()

            jump_generator.solve_prb()

            jump_generator.postproc_sol()

    # if args.opt_type == full_opt_name:

    #     # raw + resampled solution generation
    #     jump_generator = fullJumpGen(horizon_config_fullpath, 
    #                 actuators_config_fullpath, 
    #                 urdf_full_path,
    #                 args.results_dir, 
    #                 sol_mat_name = full_opt_name + "_" + args.sol_mat_name, 
    #                 sol_mat_name_res = full_opt_name + "_" + args.sol_mat_name_res, 
    #                 sol_mat_name_ref = full_opt_name + "_" + args.sol_mat_name_ref,
    #                 is_ref_prb = False, 
    #                 acc_based_formulation = args.is_acc_based_form)

    #     jump_generator.init_prb()
        
    #     jump_generator.setup_prb()

    #     jump_generator.solve_prb()

    #     jump_generator.postproc_sol()

    #     if args.refine:
    #         # refined solution generation
    #         jump_generator = fullJumpGen(horizon_config_fullpath, 
    #                     actuators_config_fullpath, 
    #                     urdf_full_path,
    #                     args.results_dir, 
    #                     sol_mat_name = full_opt_name + "_" + args.sol_mat_name, 
    #                     sol_mat_name_res = full_opt_name + "_" + args.sol_mat_name_res, 
    #                     sol_mat_name_ref = full_opt_name + "_" + args.sol_mat_name_ref,
    #                     is_ref_prb = True, 
    #                     load_ig = args.load_ig, 
    #                     acc_based_formulation = args.is_acc_based_form)

    #         jump_generator.init_prb()
            
    #         jump_generator.setup_prb()

    #         jump_generator.solve_prb()

    #         jump_generator.postproc_sol()

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(
        description='Pipeline script for the generation of offline jumps of the Awesome Leg')

    # first level specific arguments
    parser.add_argument('--urdf_path', '-urdf', type = str, default= rospkg.RosPack().get_path("awesome_leg") + "/description/urdf"+ "/" + "awesome_leg_test_rig" + ".urdf")
    
    parser.add_argument('-sol_mat_name', "-matn", type = str, default = sol_mat_basename)
    parser.add_argument('-sol_mat_name_res', "-matn_res", type = str, default = sol_mat_basename + "_res")
    parser.add_argument('-sol_mat_name_ref', "-matn_ref", type = str, default = sol_mat_basename + "_ref")

    parser.add_argument('--results_dir', '-rdir', type = str, help = 'where results are saved', default = "/tmp/" + file_name + "_" + unique_id)

    parser.add_argument('--hor_confname', '-hconf', type = str,\
                        help = 'horizon config file name', default = file_name)
                        
    parser.add_argument('--opt_type', '-optt', type = str,\
                        help = 'optimization type', default = apex_opt_name, 
                        choices = [pretakeoff_opt_name, apex_opt_name, full_opt_name])

    parser.add_argument('--is_acc_based_form', '-isacc', type = str2bool,\
                        help = 'whether to use an acceleration-based formulation or not', default = True)
    
    parser.add_argument('--refine', '-ref', type = str2bool,\
                        help = 'whether to produce also refined solutions', default = True)

    parser.add_argument('--load_ig', '-uig', type = str2bool, default = True)

    args = parser.parse_args()

    rospackage = rospkg.RosPack() # Only for taking the path to the leg package
    exec_path = rospackage.get_path("awesome_leg") + "/src/horizon_code"
    urdfs_path = rospackage.get_path("awesome_leg") + "/description/urdf"
    urdf_name = "awesome_leg_test_rig"
    urdf_full_path = urdfs_path + "/" + urdf_name + ".urdf"
    xacro_full_path = urdfs_path + "/" + urdf_name + ".urdf.xacro"
    
    sliding_guide_command = "sliding_guide:=" + "true"

    config_path=rospackage.get_path("awesome_leg")+"/config/" # configuration files path
    horizon_config_path = config_path + "horizon/jump_generation/"
    horizon_config_fullpath = horizon_config_path + args.hor_confname + ".yaml"
    actuators_config_fullpath = config_path + "actuators.yaml"

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

    os.mkdir(args.results_dir)
    shutil.copyfile(actuators_config_fullpath, args.results_dir + "/actuators" + unique_id + ".yaml") 
    shutil.copyfile(horizon_config_fullpath, args.results_dir + "/" + "horizon_config.yaml" )
    shutil.copyfile(xacro_full_path, args.results_dir + "/" + urdf_name + ".urdf.xacro" )
    shutil.copyfile(urdf_full_path, args.results_dir + "/" + urdf_name + ".urdf" )
    
    main(args)