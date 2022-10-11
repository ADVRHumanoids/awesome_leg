#!/usr/bin/env python3

from jump_utils.horizon_utils import JumpSolPlotter

plotter = JumpSolPlotter("/tmp/awesome_jump_pipeline_11-10-2022-12_57_28", 
                    sim_postproc_filename = "sim_traj_replay__2_2022_10_11__17_31_39.mat")

# make_plot_selector = [False, False, False, True]

plotter.make_sim_plots()

# plotter.make_link_comp_plots()

plotter.show_plots()