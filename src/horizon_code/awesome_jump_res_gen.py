#!/usr/bin/env python3

import argparse

import rospkg

from jump_utils.jump_tasks import JumpGen

def main(args):

    jump_generator_resampled = JumpGen(args.yaml_path, 
                    args.actuators_yaml_path, 
                    args.urdf_path,
                    args.results_dir, 
                    sol_mat_name = args.sol_mat_name, 
                    sol_mat_name_res = args.sol_mat_name_res)

    jump_generator_resampled.init_prb()
    jump_generator_resampled.setup_prb()

    jump_generator_resampled.solve_prb()

    jump_generator_resampled.postproc_sol()

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(
        description='Pipeline script for the generation of offline jumps of the Awesome Leg')

    # first level specific arguments
    parser.add_argument('--urdf_path', '-urdf', type = str, default= rospkg.RosPack().get_path("awesome_leg") + "/description/urdf"+ "/" + "awesome_leg_test_rig" + ".urdf")
    parser.add_argument('-yaml_path', "-yaml", type = str, default = rospkg.RosPack().get_path("awesome_leg")+"/config/" + "horizon/" + "awesome_jump_pipeline" + ".yaml")
    parser.add_argument('-actuators_yaml_path', "-ayaml", type = str, default = rospkg.RosPack().get_path("awesome_leg")+"/config/" + "actuators.yaml")
    parser.add_argument('-sol_mat_name', "-matn", type = str, default = "awesome_jump")
    parser.add_argument('-sol_mat_name_res', "-matn_res", type = str, default = "awesome_jump_res")
    parser.add_argument('--results_dir', '-rdir', type = str)

    args = parser.parse_args()

    main(args)
    
