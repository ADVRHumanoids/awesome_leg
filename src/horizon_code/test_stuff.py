#!/usr/bin/env python3

from jump_utils.horizon_utils import JumpSolPlotter

plotter = JumpSolPlotter("/tmp/awesome_jump_pipeline_12-10-2022-14_09_38", 
                    sim_postproc_filename = "sim_traj_replay__0_2022_10_12__14_30_00.mat")

# make_plot_selector = [False, False, False, True]

plotter.make_sim_plots()

# plotter.make_link_comp_plots()

plotter.show_plots()