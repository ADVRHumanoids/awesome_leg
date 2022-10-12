#!/usr/bin/env python3

from jump_utils.horizon_utils import JumpSolPlotter

import argparse 

if __name__ == '__main__':

    parser = argparse.ArgumentParser(
        description='Script for plotting offline jumps of the Awesome Leg')

    parser.add_argument('--sol_type', '-stype', type = str, default = "")
    parser.add_argument('--base_sol_name', '-bsname', type = str, default = "awesome_jump")
    parser.add_argument('--sol_path', '-path', type = str)
    parser.add_argument('--plot_raw', '-raw', type = bool, default = False)
    parser.add_argument('--plot_res', '-res', type = bool, default = False)
    parser.add_argument('--plot_ref', '-ref', type = bool, default = False)
    parser.add_argument('--plot_comp', '-comp', type = bool, default = False)

    args = parser.parse_args()

    plotter = JumpSolPlotter(args.sol_path)
    make_plot_selector = [args.plot_raw, args.plot_res, args.plot_ref, args.plot_comp]

    plotter.make_opt_plots(make_plot_selector)

    plotter.show_plots()