#!/usr/bin/env python3

from jump_utils.horizon_utils import JumpSolPlotter

import argparse 

if __name__ == '__main__':

    parser = argparse.ArgumentParser(
        description='Pipeline script for the generation of offline jumps of the Awesome Leg')

    parser.add_argument('--sol_type', '-stype', type = str, default = "")
    parser.add_argument('--base_sol_name', '-bsname', type = str, default = "awesome_jump")
    parser.add_argument('--sol_path', '-path', type = str)

    args = parser.parse_args()

    plotter = JumpSolPlotter(args.sol_path + "/" + args.base_sol_name + args.sol_type + ".mat")

    plotter.make_plots()

    plotter.show_plots()