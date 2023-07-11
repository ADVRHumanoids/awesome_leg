#!/usr/bin/env python3

from plot_utils.plot_utils import JumpSolPlotter
import argparse 

from plot_utils.plot_utils import str2bool

if __name__ == '__main__':

    parser = argparse.ArgumentParser(
        description='Script for plotting offline jumps of the Awesome Leg')

    parser.add_argument('--base_sol_name', '-bsname', type = str, default = "apex_awesome_jump")
    parser.add_argument('--sol_path', '-path', type = str, default = "/tmp/humanoids_opendata/opt_jumps/jump_generation_06-07-2023-14_24_01")
    parser.add_argument('--sim_name', '-sname', type = str, default = None)
    parser.add_argument('--test_name', '-tname', type = str, default = None)
    parser.add_argument('--plot_raw', '-raw', type = str2bool, default = False)
    parser.add_argument('--plot_res', '-res', type = str2bool, default = False)
    parser.add_argument('--plot_ref', '-ref', type = str2bool, default = True)
    parser.add_argument('--plot_comp_resref', '-comp_rsrf', type = str2bool, default = False)
    parser.add_argument('--plot_comp_rawres', '-comp_rwrs', type = str2bool, default = False)
    parser.add_argument('--plot_comp_rawref', '-comp_rwrf', type = str2bool, default = False)

    args = parser.parse_args()

    plotter = JumpSolPlotter(args.sol_path, 
                            sim_postproc_filename= args.sim_name, test_postproc_filename= args.test_name, 
                            opt_base_sol_name = args.base_sol_name)
    make_plot_selector = [args.plot_raw, args.plot_res, args.plot_ref, \
                            args.plot_comp_resref, args.plot_comp_rawres, args.plot_comp_rawref]
    
    plotter.make_opt_plots(make_plot_selector)

    plot_sim = True if args.sim_name is not None else False

    plotter.make_sim_plots(plot_sim)

    plot_test = True if args.test_name is not None else False

    plotter.make_test_plots(plot_test)

    plotter.show_plots()