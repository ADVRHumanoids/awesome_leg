#!/usr/bin/env python3

from jump_utils.horizon_utils import JumpSolPlotter

plotter = JumpSolPlotter("/tmp/awesome_jump_pipeline_04-10-2022-16_10_24")

make_plot_selector = [False, False, False, True]
plotter.make_plots(make_plot_selector)

plotter.show_plots()