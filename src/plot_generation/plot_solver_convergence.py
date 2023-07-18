#!/usr/bin/env python3

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm
from matplotlib.colors import LightSource, Normalize
from mpl_toolkits.axes_grid1 import make_axes_locatable

from horizon.utils import mat_storer

class LoadSolvInfo:

    def __init__(self, mat_file_path):

        ms_loader_ref = mat_storer.matStorer(mat_file_path)

        self.opt_data=ms_loader_ref.load() 

        self.opt_costs = np.array(self.opt_data["cost_values"][:, 0, 0])

        self.normalization = np.max(self.opt_costs)

        self.opt_costs_norm = self.opt_costs / self.normalization

    ## Low-level methods for manipulating the .mat file ##

jump_path = "/tmp/humanoids_opendata/opt_jumps/jump_generation_06-07-2023-14_24_01"
landing_path = "/tmp/humanoids_opendata/opt_landings/touchdown_opt_11-07-2023-12_25_45"
jump_mat_name = "apex_awesome_jump_ref"
jump_mat_name_base = "apex_awesome_jump"

landing_mat_name = "energy_recov_ig"

jump_data_log = LoadSolvInfo(jump_path + "/" + jump_mat_name + ".mat")
jump_data_log_base = LoadSolvInfo(jump_path + "/" + jump_mat_name_base + ".mat")

landing_data_log = LoadSolvInfo(landing_path + "/" + landing_mat_name + ".mat")

fontsize = 14
f1, ax = plt.subplots(1)

ax.plot(jump_data_log_base.opt_costs_norm, drawstyle='steps-post')
ax.plot(jump_data_log.opt_costs_norm, drawstyle='steps-post')

ax.set_xlabel('iteration n.', fontsize=fontsize)
ax.set_ylabel('', fontsize=fontsize)
ax.set_title('Take-off optimizaton convergence - Ipopt - ma57', fontsize=fontsize)
ax.grid()

legend = ax.legend([f"normalized cost - base take-off opt", 
                    f"normalized cost - refined take-off opt"], fontsize=fontsize)
legend.set_draggable(state = True)

f1.set_figheight(5)
f1.set_figwidth(10)

fontsize = 14
f2, ax2 = plt.subplots(1)

ax2.plot(landing_data_log.opt_costs_norm, drawstyle='steps-post')

ax2.set_xlabel('iteration n.', fontsize=fontsize)
ax2.set_ylabel('', fontsize=fontsize)
ax2.set_title('Take-off and landing optimization convergence - Ipopt - ma57', fontsize=fontsize)
ax2.grid()

legend = ax2.legend([f"normalized cost - landing opt"], fontsize=fontsize)
legend.set_draggable(state = True)

f2.set_figheight(5)
f2.set_figwidth(10)

fontsize = 14
f3, ax3 = plt.subplots(1)

ax3.plot(jump_data_log_base.opt_costs_norm, drawstyle='steps-post')
ax3.plot(jump_data_log.opt_costs_norm, drawstyle='steps-post')
ax3.plot(landing_data_log.opt_costs_norm, drawstyle='steps-post')

# ax3[0].set_xlabel('iteration n.', fontsize=fontsize)
ax3.set_ylabel('', fontsize=fontsize)
ax3.set_xlabel('iteration n.', fontsize=fontsize)

ax3.set_title('Take-off and landing optimization convergence - Ipopt - ma57', fontsize=fontsize)
ax3.grid()

legend = ax3.legend([f"normalized cost - base take-off opt", 
                    f"normalized cost - refined take-off opt", 
                    f"normalized cost - landing opt"], fontsize=fontsize)
legend.set_draggable(state = True)

# ax3[1].plot(landing_data_log.opt_costs_norm, drawstyle='steps-post')

# ax3[1].set_xlabel('iteration n.', fontsize=fontsize)
# ax3[1].set_ylabel('', fontsize=fontsize)
# ax3[1].set_title('Landing optimization convergence - Ipopt - ma57', fontsize=fontsize)
# ax3[1].grid()

# legend = ax3[1].legend([f"normalized cost - landing opt"], fontsize=fontsize)
# legend.set_draggable(state = True)

# f3.set_figheight(5)
# f3.set_figwidth(10)

plt.show()