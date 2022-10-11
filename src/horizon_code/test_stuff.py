#!/usr/bin/env python3

from jump_utils.horizon_utils import JumpSolPlotter

plotter = JumpSolPlotter("/tmp/awesome_jump_pipeline_10-10-2022-21_44_30", 
                    sim_postproc_filename = "sim_traj_replay__0_2022_10_10__18_25_29.mat", 
                    test_postproc_filename="sim_traj_replay__0_2022_10_10__18_25_29.mat")

make_plot_selector = [False, False, False, True]

# plotter.make_sim_plots()

# plotter.make_link_comp_plots()

plotter.show_plots()