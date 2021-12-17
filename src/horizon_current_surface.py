#!/usr/bin/env python3

###############################################
import rospy
import rospkg

from horizon.utils import mat_storer

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import numpy as np
###############################################

rospackage=rospkg.RosPack()
ms_loaded = mat_storer.matStorer(rospackage.get_path("awesome_leg_pholus")+"/opt_results/horizon_offline_solver.mat")
solution=ms_loaded.load() # loading the solution dictionary

q_p_ddot=solution["q_p_ddot"]
tau=solution["tau"]
dt = rospy.get_param("/horizon_solver/constant_dt/problem_settings/dt")   # optimization dt [s]

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

fig_hip, ax_hip = plt.subplots(subplot_kw={"projection": "3d"})
fig_knee, ax_knee = plt.subplots(subplot_kw={"projection": "3d"})

red_ratio_lb_hip=0.01
red_ratio_ub_hip=0.1
n_eff_samples_hip=100
red_ratio_lb_knee=0.01
red_ratio_ub_knee=0.1
n_eff_samples_knee=100

###############################################

Efficiency_samples_hip = np.linspace(red_ratio_lb_hip, red_ratio_ub_hip, n_eff_samples_hip)
Efficiency_samples_knee = np.linspace(red_ratio_lb_knee, red_ratio_ub_knee, n_eff_samples_knee)

X_hip, Eff_hip = np.meshgrid(time_vector[:-1], Efficiency_samples_hip)
X_knee, Eff_knee = np.meshgrid(time_vector[:-1], Efficiency_samples_knee)

Q_p_ddot_hip, _ = np.meshgrid(q_p_ddot[0,:], Efficiency_samples_hip)
Q_p_ddot_knee, _ = np.meshgrid(q_p_ddot[1,:], Efficiency_samples_knee)
Tau_p_ddot_hip, _ = np.meshgrid(tau[0,:], Efficiency_samples_hip)
Tau_p_ddot_knee, _ = np.meshgrid(tau[1,:], Efficiency_samples_knee)

I_hip = np.absolute((hip_axial_MoI*Q_p_ddot_hip/Eff_hip+Tau_p_ddot_hip*Eff_hip/hip_efficiency)*1.0/hip_K_t)
I_knee = np.absolute((knee_axial_MoI*Q_p_ddot_knee/Eff_knee+Tau_p_ddot_knee*Eff_knee/knee_efficiency)*1.0/knee_K_t)

surf_hip = ax_hip.plot_surface(X_hip, Eff_hip, I_hip, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)

surf_knee = ax_knee.plot_surface(X_knee, Eff_knee, I_knee, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)

ax_hip.zaxis.set_major_locator(LinearLocator(10))
ax_hip.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
ax_hip.set_xlabel('$t$', fontsize=10)
ax_hip.set_ylabel('$\mu$', fontsize=10)
ax_hip.set_zlabel('$i_q$', fontsize=10)

ax_knee.zaxis.set_major_locator(LinearLocator(10))
ax_knee.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
ax_knee.set_xlabel('$t$', fontsize=10)
ax_knee.set_ylabel('$\mu$', fontsize=10)
ax_knee.set_zlabel('$i_q$', fontsize=10)

# Add a color bar which maps values to colors.
fig_hip.colorbar(surf_hip, shrink=0.5, aspect=5)
fig_knee.colorbar(surf_knee, shrink=0.5, aspect=5)

ax_hip.set_title('Required hip $i_q$')
ax_knee.set_title('Required knee $i_q$')

plt.show()
