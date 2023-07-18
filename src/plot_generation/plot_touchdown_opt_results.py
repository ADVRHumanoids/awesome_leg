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
landing_path2 = "/tmp/humanoids_opendata/opt_landings/touchdown_opt_14-07-2023-11_31_13_no_critical_damp"

landing_mat_name = "energy_recov"

landing_data_log = LoadOptLanding(landing_path + "/" + landing_name + ".mat")
landing_data_log2 = LoadOptLanding(landing_path2 + "/" + landing_name + ".mat")

dt_opt = landing_data_log.opt_data["dt_opt"]

p_batt = landing_data_log.opt_data["p_batt"]
p_joule = landing_data_log.opt_data["r_iq_2_sol"]
p_mech = landing_data_log.opt_data["t_w_sol"]
p_ind_f = landing_data_log.opt_data["l_iq_f_sol"]
p_ind_i = landing_data_log.opt_data["l_iq_i_sol"]

p_batt2 = landing_data_log2.opt_data["p_batt"]
p_joule2 = landing_data_log2.opt_data["r_iq_2_sol"]
p_mech2 = landing_data_log2.opt_data["t_w_sol"]
p_ind_f2 = landing_data_log2.opt_data["l_iq_f_sol"]
p_ind_i2 = landing_data_log2.opt_data["l_iq_i_sol"]

q_landing = landing_data_log.opt_data["q_landing"]
tip_vel_impact = landing_data_log.opt_data["q_p_dot"][0, 0]
cost_vals = landing_data_log.opt_data["cost_values"]
e_batt = landing_data_log.opt_data["e_batt"]
f_contact = landing_data_log.opt_data["f_contact"]
impact = landing_data_log.opt_data["impact"]
iq_check = landing_data_log.opt_data["iq_check"]
tau_so = landing_data_log.opt_data["tau_sol"]
kd = landing_data_log.opt_data["kd"]
kp = landing_data_log.opt_data["kp"]

q_landing2 = landing_data_log2.opt_data["q_landing"]
tip_vel_impact2 = landing_data_log2.opt_data["q_p_dot"][0, 0]
cost_vals2 = landing_data_log2.opt_data["cost_values"]
e_batt2 = landing_data_log2.opt_data["e_batt"]
f_contact2 = landing_data_log2.opt_data["f_contact"]
impact2 = landing_data_log2.opt_data["impact"]
iq_check2 = landing_data_log2.opt_data["iq_check"]
tau_so2 = landing_data_log2.opt_data["tau_sol"]
kd2 = landing_data_log2.opt_data["kd"]
kp2 = landing_data_log2.opt_data["kp"]

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

# critically damped p batt

fontsize = 14
fig2, ax2 = plt.subplots(2)

ax2[0].plot(time_vect, p_batt.flatten())

# ax2[0].set_xlabel('time', fontsize=fontsize)
ax2[0].set_ylabel('[W]', fontsize=fontsize)
ax2[0].set_title('Touchdown opt with energy recovery maximization', fontsize=fontsize)
ax2[0].grid()

impact1_ratio = impact[2][0] / tip_vel_impact
description1 = [f"(final optimal solution)" + f"\n" + \
                r"$e_{\mathrm{batt}}: $" + f"{round(e_batt[0][0], 2)}" + r"$~\mathrm{J}$" + f"\n" + \
                r"$\rho: $" + f"{round(impact1_ratio, 2)}" + r"$~\mathrm{N\,s^2}/\mathrm{m}$" + f"\n" + \
                r"$K_p: $ " + f"[{round(kp[0, 0], 2)}" + f", {round(kp[1, 0], 2)}]" + r"$~\mathrm{N\,m}/\mathrm{rad}$" + f"\n"+ \
                r"$K_d: $ " + f"[{round(kd[0, 0], 2)}" + f", {round(kd[1, 0], 2)}]" + r"$~\mathrm{N\,m}/\mathrm{rad}*s$"]
legend1 = ax2[0].legend(description1, loc='upper right', handlelength=0, handletextpad=0, fancybox=True, fontsize = fontsize)
legend1.set_draggable(state = True)

ax2[1].plot(time_vect, p_batt2.flatten())

ax2[1].set_xlabel('time [s]', fontsize=fontsize)
ax2[1].set_ylabel('[W]', fontsize=fontsize)
ax2[1].set_title('Touchdown opt without energy recovery maximization', fontsize=fontsize)
ax2[1].grid()

impact2_ratio = impact2[2][0] / tip_vel_impact2
description2 = [r"$e_{\mathrm{batt}}: $" + f"{round(e_batt2[0][0], 2)}" + r"$ ~\mathrm{J}$" + f"\n" + \
            r"$\rho: $" + f"{round(impact2_ratio, 2)}" + r"$~\mathrm{N\,s^2}/\mathrm{m}$" + f"\n" + \
                r"$K_p: $ " + f"[{round(kp2[0, 0], 2)}" + f", {round(kp2[1, 0], 2)}]" + r"$~\mathrm{N\,m}/\mathrm{rad}$" + f"\n"+ \
                r"$K_d: $ " + f"[{round(kd2[0, 0], 2)}" + f", {round(kd2[1, 0], 2)}]" + r"$~\mathrm{N\,m}/\mathrm{rad}*s$"]

legend2 = ax2[1].legend(description2, loc='upper right', handlelength=0, handletextpad=0, fancybox=True, fontsize = fontsize)
legend2.set_draggable(state = True)

plt.show()