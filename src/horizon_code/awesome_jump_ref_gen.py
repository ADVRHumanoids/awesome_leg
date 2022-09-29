#!/usr/bin/env python3
import os, argparse

import rospkg

from datetime import datetime
from datetime import date

import subprocess

from termcolor import colored

import numpy as np

from codesign_pyutils.miscell_utils import str2bool

from codesign_pyutils.post_proc_utils import PostProcS3

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(
        description='Pipeline script for the co-design of RePAIR project')

    # first level specific arguments
    parser.add_argument('--multistart_n_l1', '-msn_l1', type = int,\
                        help = '', default = 108)
    parser.add_argument('--max_trials_factor_l1', '-mtfl1', type=int,\
                        help = 'for each multistart node, at best max_trials_factor new solutions will be tried to obtain an optimal solution',
                        default = 10)

    parser.add_argument('--is_in_place_flip', '-iplf', type=str2bool,\
                        help = 'whether to use in place flip task', default = True)
    parser.add_argument('--is_biman_pick', '-ibp', type=str2bool,\
                        help = 'whether to use bimanual pick task', default = True)

    parser.add_argument('--ig_seed_l1', '-ig_l1', type = int,\
                        help = '', default = 547)
    parser.add_argument('--ipopt_verb_lev', '-ipopt_v', type = int,\
                        help = '', default = 1)
    parser.add_argument('--filling_nnodes', '-fnn', type = int,\
                        help = '', default = 3)
    parser.add_argument('--use_ma57', '-ma57', type=str2bool,\
                        help = 'whether to use ma57 linear solver or not', default = False)
    parser.add_argument('--wrist_offset', '-wo', type = np.double,\
                        help = 'sliding_wrist_offset', default = 0.0)
    parser.add_argument('--is_sliding_wrist', '-isw', type = bool,\
                        help = 'is wrist off. is to be used as an additional codes variable', default = True)

    parser.add_argument('--n_y_samples_flip', '-nysf', type = int,\
                        help = 'number of y-axis samples on which tasks (flipping task) are placed', default = 3)
    parser.add_argument('--y_sampl_ub_flip', '-yubf', type = np.double,\
                        help = 'upper bound of the y sampling (bimanual task)', default = 0.3)
    parser.add_argument('--n_y_samples_biman', '-nysb', type = int,\
                        help = 'number of y-axis samples on which tasks(flipping task) are placed', default = 3)
    parser.add_argument('--y_sampl_ub_biman', '-yubb', type = np.double,\
                        help = 'upper bound of the y sampling (bimanual task)', default = 0.2)
                        
    # second level-specific arguments
    parser.add_argument('--multistart_n_l2', '-msn_l2', type=int,\
                        help = 'number of multistarts (per cluster) to use',
                        default = 20)
    parser.add_argument('--max_trials_factor_l2', '-mtfl2', type=int,\
                        help = 'for each multistart node, at best max_trials_factor new solutions will be tried to obtain an optimal solution',
                        default = 20)
    parser.add_argument('--n_clust_l2', '-nc_l2', type=int,\
                        help = 'number of clusters to be generated', default = 5)
    parser.add_argument("--ig_seed_l2", '-ig_l2', type = int,\
                        help = '', default = 129)
    
    # cl man reference generation-specific arguments
    # parser.add_argument('--gen_cl_man_ref', '-gen_clmr', type=bool,\
    #                     help = 'whether to run the cl. manipulability reference generation',
    #                     default = True)
    # parser.add_argument('--max_trials_factor_clmr', '-mtfl2', type=int,\
    #                     help = 'for each multistart node, at best max_trials_factor new solutions will be tried to obtain an optimal solution',
    #                     default = 20)

    args = parser.parse_args()


    # unique id used for generation of results
    unique_id = date.today().strftime("%d-%m-%Y") + "-" +\
                        datetime.now().strftime("%H_%M_%S")

    # useful paths
    l1_dump_folder_name = "first_level"
    l2_dump_folder_name = "second_level"
    res_dir_basename = "test_results"
    res_dir_full_name = res_dir_basename + "_" + \
                unique_id
    rospackage = rospkg.RosPack() # Only for taking the path to the leg package

    codesign_path = rospackage.get_path("repair_codesign")
    exec_path = codesign_path + "/src"

    l1_results_path = codesign_path + "/" + res_dir_basename + "/" + res_dir_full_name + "/" + l1_dump_folder_name
    l2_results_path = codesign_path + "/" + res_dir_basename + "/" + res_dir_full_name + "/" + l2_dump_folder_name

    #generating urdf
    urdfs_path = rospackage.get_path("repair_urdf") + "/urdf"
    urdf_name = "repair_full"
    urdf_full_path = urdfs_path + "/" + urdf_name + ".urdf"
    xacro_full_path = urdfs_path + "/" + urdf_name + ".urdf.xacro"
    sliding_wrist_command = "is_sliding_wrist:=" + "true"
    gen_coll_command = "gen_coll:=" + "true"

    coll_yaml_name = "arm_coll.yaml"
    coll_yaml_path = rospackage.get_path("repair_urdf") + "/config/" + coll_yaml_name

    try:

        print(colored("\n--> GENERATING URDF...\n", "blue"))
        xacro_gen = subprocess.check_call(["xacro",\
                                        xacro_full_path, \
                                        sliding_wrist_command, \
                                        gen_coll_command, \
                                        "-o", 
                                        urdf_full_path])

        print(colored("\n--> URDF GENERATED SUCCESSFULLY. \n", "blue"))

    except:

        print(colored('FAILED TO GENERATE URDF.', "red"))


    os.chdir(exec_path) # change current path, so that executable can be run with check_call

    if args.multistart_n_l1 > 0:

        try:

            print(colored("\n--> STARTING FIRST LEVEL OPTIMIZATION....\n", "blue"))
            reset_term = subprocess.check_call(["reset"])
            # run first level (blocking --> we have to wait for data to be dumped to file)
            first_level_proc = subprocess.check_call(["./run_1st_step_opt_on_workstation.py", \
                                        "-mst", \
                                        str(args.multistart_n_l1), \
                                        "-mtf", \
                                        str(args.max_trials_factor_l1), \
                                        "-igs", \
                                        str(args.ig_seed_l1),  \
                                        "-fnn", \
                                        str(args.filling_nnodes), \
                                        "-ipopt_v", \
                                        str(args.ipopt_verb_lev), \
                                        "-run_ext", \
                                        str(True), \
                                        "-id", \
                                        str(unique_id), \
                                        "-ma57", \
                                        str(args.use_ma57), \
                                        "-wo", \
                                        str(args.wrist_offset), \
                                        "-isw", \
                                        str(args.is_sliding_wrist), \
                                        "-nysf", \
                                        str(args.n_y_samples_flip),\
                                        "-nysb", \
                                        str(args.n_y_samples_biman),\
                                        "-yubf", \
                                        str(args.y_sampl_ub_flip), \
                                        "-yubb", \
                                        str(args.y_sampl_ub_biman), \
                                        "-dfn", \
                                        l1_dump_folder_name, \
                                        "-rdbs", \
                                        res_dir_basename, \
                                        "-iplf", \
                                        str(args.is_in_place_flip), \
                                        "-ibp", \
                                        str(args.is_biman_pick), \
                                        "-urdf", \
                                        urdf_full_path, \
                                        "-coll", \
                                        coll_yaml_path])

            print(colored("\n--> FIRST LEVEL OPTIMIZATION FINISHED SUCCESSFULLY. \n", "blue"))

        except:

            print(colored('\n An exception occurred while running the first level of the codesign pipeline. Muy malo!!! \n', "red"))
    

    if args.multistart_n_l2 > 0:

        try:

            print(colored("\n--> STARTING SECOND LEVEL OPTIMIZATION....\n", "blue"))

            #run first level (blocking --> we have to wait for data to be dumped to file)
            second_level_proc = subprocess.check_call(["./run_2nd_3rd_step_opt_on_workstation.py", \
                                        "-d", \
                                        res_dir_full_name, \
                                        "-dfn", \
                                        l2_dump_folder_name,
                                        "-rdbs", \
                                        res_dir_basename, \
                                        "-ldn", \
                                        l1_dump_folder_name, \
                                        "-ipopt_v", \
                                        str(args.ipopt_verb_lev), \
                                        "-nc",\
                                        str(args.n_clust_l2), 
                                        "-ma57", \
                                        str(args.use_ma57), 
                                        "-igs", \
                                        str(args.ig_seed_l2),
                                        "-mtf", \
                                        str(args.max_trials_factor_l2), \
                                        "-mst",
                                        str(args.multistart_n_l2), \
                                        "-urdf", \
                                        urdf_full_path, \
                                        "-coll", \
                                        coll_yaml_path])

            print(colored("\n--> SECOND LEVEL OPTIMIZATION FINISHED SUCCESSFULLY. \n", "blue"))

        except:

            print(colored('\n An exception occurred while running the second level of the codesign pipeline. Muy malo!!! \n', "red"))
    
