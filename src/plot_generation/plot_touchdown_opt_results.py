#!/usr/bin/env python3

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm
from matplotlib.colors import LightSource, Normalize
from mpl_toolkits.axes_grid1 import make_axes_locatable

from horizon.utils import mat_storer

class LoadOptLanding:

    def __init__(self, mat_file_path):

        ms_loader_ref = mat_storer.matStorer(mat_file_path)

        self.opt_data=ms_loader_ref.load() 

        self.opt_costs = np.array(self.opt_data["cost_values"][:, 0, 0])

        self.normalization = np.max(self.opt_costs)

        self.opt_costs_norm = self.opt_costs / self.normalization

    ## Low-level methods for manipulating the .mat file ##

landing_path = "/tmp/humanoids_opendata/opt_landings/touchdown_opt_11-07-2023-12_25_45"
landing_name = "energy_recov_ig"


landing_mat_name = "energy_recov"

landing_data_log = LoadOptLanding(landing_path + "/" + landing_name + ".mat")

dt_opt = landing_data_log.opt_data["dt_opt"]

p_batt = landing_data_log.opt_data["p_batt"]
p_joule = landing_data_log.opt_data["r_iq_2_sol"]
p_mech = landing_data_log.opt_data["t_w_sol"]
p_ind_f = landing_data_log.opt_data["l_iq_f_sol"]
p_ind_i = landing_data_log.opt_data["l_iq_i_sol"]

q_landing = landing_data_log.opt_data["q_landing"]
cost_vals = landing_data_log.opt_data["cost_values"]
e_batt = landing_data_log.opt_data["e_batt"]
f_contact = landing_data_log.opt_data["f_contact"]
impact = landing_data_log.opt_data["impact"]
iq_check = landing_data_log.opt_data["iq_check"]
tau_so = landing_data_log.opt_data["tau_sol"]
kd = landing_data_log.opt_data["kd"]
kp = landing_data_log.opt_data["kp"]

time_vect = np.zeros((p_batt.shape[1]))
for i in range(0, len(time_vect) - 1):
    time_vect[i + 1] = time_vect[i] + dt_opt

# power towards batt
fontsize = 14
fig, ax = plt.subplots(1)

ax.plot(time_vect, p_batt.flatten())
ax.plot(time_vect, p_joule.flatten())
ax.plot(time_vect, p_mech.flatten())

ax.set_xlabel('time', fontsize=fontsize)
ax.set_ylabel('[W]', fontsize=fontsize)
ax.set_title('Optimized battery power flow during landing', fontsize=fontsize)
ax.grid()

legend = ax.legend([r"$\mathrm{p}_{\mathrm{batt}}$", 
                    r"$\mathrm{p}_{\mathrm{joule}}$", 
                    r"$\mathrm{p}_{\mathrm{mech}}$"], fontsize=fontsize)
legend.set_draggable(state = True)

plt.show()