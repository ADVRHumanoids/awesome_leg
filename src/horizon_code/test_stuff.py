#!/usr/bin/env python3

from jump_utils.horizon_utils import JumpSolPlotter

plotter = JumpSolPlotter("/home/andreap/hhcm_ws/src/awesome_leg/opt_results/horizon_jump/jump_test/awesome_jump_pipeline_05-10-2022-18_54_07", 
                    sim_postproc_filename = "sim_traj_replay__0_2022_10_10__17_10_45.mat", 
                    test_postproc_filename="sim_traj_replay__0_2022_10_10__17_10_45.mat")

make_plot_selector = [False, False, False, True]
# plotter.make_opt_plots(make_plot_selector)

plotter.make_sim_plots()

plotter.make_link_comp_plots()

plotter.show_plots()