#!/usr/bin/env python3

from jump_utils.horizon_utils import JumpSolPlotter

plotter = JumpSolPlotter("/tmp/awesome_jump_pipeline_01-10-2022-18_51_09/awesome_jump_ref.mat")

plotter.make_plots()

plotter.show_plots()