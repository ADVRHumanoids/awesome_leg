#!/usr/bin/env python3

###############################################
import rospy
import rospkg

from horizon.utils import mat_storer

import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter

import numpy as np
###############################################

rospackage=rospkg.RosPack()
ms_loaded = mat_storer.matStorer(rospackage.get_path("awesome_leg_pholus")+"/sim_results/horizon_offline_solver.mat")
ms_test = mat_storer.matStorer(rospackage.get_path("awesome_leg_pholus")+"/sim_results/horizon_offline_solver.mat")
solution=ms_loaded.load() # loading the solution dictionary

q_p=solution["q_p"]
q_p_dot=solution["q_p_dot"]
q_p_ddot=solution["q_p_ddot"]
i_q=solution["i_q"]
GRF=solution["f_contact"]
tau=solution["tau"]
dt = rospy.get_param("/horizon/problem/dt")   # optimization dt [s]

time_vector = np.zeros([tau[0,:].size+1])
for i in range(tau[0,:].size):
    time_vector[i+1] = time_vector[i] + dt

# hip actuator
hip_axial_MoI=rospy.get_param("/actuators/hip/axial_MoI")
hip_efficiency=rospy.get_param("/actuators/hip/efficiency")
hip_K_t=rospy.get_param("/actuators/hip/K_t")

# knee actuator
knee_axial_MoI=rospy.get_param("/actuators/knee/axial_MoI")
knee_efficiency=rospy.get_param("/actuators/knee/efficiency")
knee_K_t=rospy.get_param("/actuators/knee/K_t")

########################## MANUAL PLOTS ##########################

plt.plot(time_vector[1:-1], GRF[0, :-1], label=r"F_x")
plt.plot(time_vector[1:-1], GRF[1, :-1], label=r"F_y")
plt.plot(time_vector[1:-1], GRF[2, :-1], label=r"F_z")
plt.legend(loc="upper left")
plt.xlabel(r"t [s]")
plt.ylabel('[N]')
plt.title("Ground reaction forces", fontdict=None, loc='center')
plt.grid()

plt.figure()
plt.plot(time_vector[:-1], q_p[0, :-1], label=r"$\mathrm{q}_{\mathrm{hip}}$")
plt.plot(time_vector[:-1], q_p[1, :-1], label=r"$\mathrm{q}_{\mathrm{knee}}$")
plt.legend(loc="upper left")
plt.xlabel(r"t [s]")
plt.ylabel(r"[rad]")
plt.title("Joint positions", fontdict=None, loc='center')
plt.grid()

plt.figure()
plt.plot(time_vector[:-1], q_p_dot[0, :-1], label=r"$\dot{q}_{\mathrm{hip}}$")
plt.plot(time_vector[:-1], q_p_dot[1, :-1], label=r"$\dot{q}_{\mathrm{knee}}$")
plt.legend(loc="upper left")
plt.xlabel(r"t [s]")
plt.ylabel(r"[rad/s]")
plt.title("Joint velocities", fontdict=None, loc='center')
plt.grid()

plt.figure()
plt.plot(time_vector[:-1], q_p_ddot[0, :], label=r"$\ddot{\mathrm{q}}_{\mathrm{hip}}$")
plt.plot(time_vector[:-1], q_p_ddot[1, :], label=r"$\ddot{\mathrm{q}}_{\mathrm{knee}}$")
plt.legend(loc="upper left")
plt.xlabel(r"t [s]")
plt.ylabel(r"[$\mathrm{rad}/\mathrm{s}^2$]")
plt.title("Joint accelerations", fontdict=None, loc='center')
plt.grid()

plt.figure()
plt.plot(time_vector[:-1], tau[0, :], label=r"$\tau_{\mathrm{hip}}$ torque")
plt.plot(time_vector[:-1], tau[1, :], label=r"$\tau_{\mathrm{knee}}$ torque")
plt.legend(loc="upper left")
plt.xlabel(r"t [s]")
plt.ylabel(r"[Nm]")
plt.title("Joint efforts", fontdict=None, loc='center')
plt.grid()

plt.figure()
plt.plot(time_vector[:-1], i_q[0, :], label=r"$\mathrm{i}_\mathrm{q}^{\mathrm{hip}}$ torque")
plt.plot(time_vector[:-1], i_q[1, :], label=r"$\mathrm{i}_\mathrm{q}^{\mathrm{knee}}$ torque")
plt.legend(loc="upper left")
plt.xlabel(r"t [s]")
plt.ylabel(r"[A]")
plt.title("Actuators direct current", fontdict=None, loc='center')
plt.grid()

plt.show()
