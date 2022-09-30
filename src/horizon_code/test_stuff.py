#!/usr/bin/env python3

from jump_utils.horizon_utils import JumpSolPlotter

plotter = JumpSolPlotter("/tmp/awesome_jump_pipeline_30-09-2022-17_59_42/awesome_jump.mat")

plotter.make_plots()

plotter.show_plots()