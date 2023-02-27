#!/usr/bin/env python3

from horizon.utils import kin_dyn
import numpy as np

from horizon.utils import mat_storer

import matplotlib.pyplot as plt
# from matplotlib import cm
# from matplotlib.ticker import LinearLocator, FormatStrFormatter


path = '/tmp/touchdown_opt/energy_recov.mat'
loader = mat_storer.matStorer(path)
solution_raw = loader.load()


n_int = solution_raw["n_int"][0][0]
q_p_raw = solution_raw["q_p"]  # excluding test rig dof
q_p_dot_raw = solution_raw["q_p_dot"]
q_p_ddot_raw = solution_raw["q_p_ddot"]
# i_q_raw = solution_raw["i_q"]
GRF_raw = solution_raw["f_contact"]
tau_raw = solution_raw["tau_sol"]
dt_raw = solution_raw["dt_landing"].flatten()
kp_raw = solution_raw['kp']
kd_raw = solution_raw['kd']
q_init_raw = solution_raw['q_landing']

print('q: ', q_init_raw)
print('kp: \n', kp_raw)
print('kd: \n', kd_raw)
print('dt:', dt_raw)
print('lambda_inv_check: ', solution_raw['lambda_inv_check'][0][0])
print('impact: ', solution_raw['impact'])

time_vector_raw = np.zeros([n_int + 1])
for i in range(dt_raw.size):
    time_vector_raw[i + 1] = time_vector_raw[i] + dt_raw[i]
    
f1 = plt.figure()
plt.plot(time_vector_raw[1:-1], GRF_raw[0, :-1], label=r"F_x")
plt.plot(time_vector_raw[1:-1], GRF_raw[1, :-1], label=r"F_y")
plt.plot(time_vector_raw[1:-1], GRF_raw[2, :-1], label=r"F_z")
legend = plt.legend(loc="upper left")
legend.set_draggable(state=True)
plt.xlabel(r"t [s]")
plt.ylabel('[N]')
plt.title("Ground reaction forces - raw solution", fontdict=None, loc='center')
plt.grid()
# if save_fig:
#     plt.savefig(save_path+"GRF.pdf", format="pdf")

f2 = plt.figure()
plt.plot(time_vector_raw[:-1], q_p_raw[0, :-1], label=r"$\mathrm{q}_{\mathrm{base}}$")
plt.plot(time_vector_raw[:-1], q_p_raw[1, :-1], label=r"$\mathrm{q}_{\mathrm{hip}}$")
plt.plot(time_vector_raw[:-1], q_p_raw[2, :-1], label=r"$\mathrm{q}_{\mathrm{knee}}$")
legend = plt.legend(loc="upper left")
legend.set_draggable(state=True)
plt.xlabel(r"t [s]")
plt.ylabel(r"[rad]")
plt.title("Joint positions - raw solution", fontdict=None, loc='center')
plt.grid()

f3 = plt.figure()
plt.plot(time_vector_raw[:-1], q_p_dot_raw[0, :-1], label=r"$\dot{q}_{\mathrm{base}}$")
plt.plot(time_vector_raw[:-1], q_p_dot_raw[1, :-1], label=r"$\dot{q}_{\mathrm{hip}}$")
plt.plot(time_vector_raw[:-1], q_p_dot_raw[2, :-1], label=r"$\dot{q}_{\mathrm{knee}}$")
legend = plt.legend(loc="upper left")
legend.set_draggable(state=True)
plt.xlabel(r"t [s]")
plt.ylabel(r"[rad/s]")
plt.title("Joint velocities - raw solution", fontdict=None, loc='center')
plt.grid()

f4 = plt.figure()
plt.plot(time_vector_raw[:-1], q_p_ddot_raw[0, :],
         label=r"$\ddot{\mathrm{q}}_{\mathrm{hip}}$")
plt.plot(time_vector_raw[:-1], q_p_ddot_raw[1, :],
         label=r"$\ddot{\mathrm{q}}_{\mathrm{knee}}$")
legend = plt.legend(loc="upper left")
legend.set_draggable(state=True)
plt.xlabel(r"t [s]")
plt.ylabel(r"[$\mathrm{rad}/\mathrm{s}^2$]")
plt.title("Joint accelerations - raw solution", fontdict=None, loc='center')
plt.grid()

f5 = plt.figure()
plt.plot(time_vector_raw[:-1], tau_raw[0, :], drawstyle='steps-post', label=r"$\tau_{\mathrm{base}}$ torque")
plt.plot(time_vector_raw[:-1], tau_raw[1, :], drawstyle='steps-post', label=r"$\tau_{\mathrm{hip}}$ torque")
plt.plot(time_vector_raw[:-1], tau_raw[2, :], drawstyle='steps-post', label=r"$\tau_{\mathrm{knee}}$ torque")
legend = plt.legend(loc="upper left")
legend.set_draggable(state=True)
plt.xlabel(r"t [s]")
plt.ylabel(r"[Nm]")
plt.title("Joint efforts (link side) - raw solution", fontdict=None, loc='center')
plt.grid()

plt.show()
# print(time_vector_raw[:-1].shape)
# print(dt_raw.shape)
#
# f8 = plt.figure()
# plt.plot(time_vector_raw[:-1], dt_raw, drawstyle='steps-post')
# legend = plt.legend(loc="upper left")
# legend.set_draggable(state=True)
# plt.ylabel(r"t [s]")
# plt.xlabel("node number")
# plt.title("Optimization dt - raw solution", fontdict=None, loc='center')
# plt.grid()

