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
do_save_fig=True

is_adaptive_dt = rospy.get_param("/horizon_solver/is_adaptive_dt")  # if true, use an adaptive dt
is_single_dt = rospy.get_param("/horizon_solver/is_single_dt")  # if true (and if addaptive dt is enable), use only one dt over the entire opt. horizon 

rospackage=rospkg.RosPack()
ms_loaded = mat_storer.matStorer(rospackage.get_path("awesome_leg_pholus")+"/sim_results/horizon_offline_solver.mat")
solution=ms_loaded.load() # loading the solution dictionary

if is_adaptive_dt:
    if is_single_dt:
        save_path=rospackage.get_path("awesome_leg_pholus")+"/media/horizon_jump/16_12_21/single_adaptive_dt/"
    else:
        save_path=rospackage.get_path("awesome_leg_pholus")+"/media/horizon_jump/16_12_21/adaptive_dt/"
else:
    save_path=rospackage.get_path("awesome_leg_pholus")+"/media/horizon_jump/16_12_21/fixed_dt/"

q_p=solution["q_p"]
q_p_dot=solution["q_p_dot"]
q_p_ddot=solution["q_p_ddot"]
i_q=solution["i_q"]
GRF=solution["f_contact"]
tau=solution["tau"]
dt=solution["dt_opt"].flatten()
hip_height=solution["hip_height"]
foot_tip_height=solution["foot_tip_height"]

time_vector = np.zeros([tau[0,:].size+1])
for i in range(tau[0,:].size):
    time_vector[i+1] = time_vector[i] + dt[i]

# hip actuator
hip_axial_MoI=rospy.get_param("/actuators/hip/axial_MoI")
hip_efficiency=rospy.get_param("/actuators/hip/efficiency")
hip_K_t=rospy.get_param("/actuators/hip/K_t")
hip_red_ratio=rospy.get_param("/actuators/hip/red_ratio")

# knee actuator
knee_axial_MoI=rospy.get_param("/actuators/knee/axial_MoI")
knee_efficiency=rospy.get_param("/actuators/knee/efficiency")
knee_K_t=rospy.get_param("/actuators/knee/K_t")
knee_red_ratio=rospy.get_param("/actuators/hip/red_ratio")

P_hip=(hip_axial_MoI*q_p_ddot[0,:]/hip_red_ratio+tau[0,:]*hip_red_ratio/hip_efficiency)*q_p_dot[0,:-1]/hip_red_ratio
P_knee=(knee_axial_MoI*q_p_ddot[1,:]/knee_red_ratio+tau[1,:]*knee_red_ratio/knee_efficiency)*q_p_dot[1,:-1]/hip_red_ratio

########################## MANUAL PLOTS ##########################
f1=plt.figure()
plt.plot(time_vector[1:-1], GRF[0, :-1], label=r"F_x")
plt.plot(time_vector[1:-1], GRF[1, :-1], label=r"F_y")
plt.plot(time_vector[1:-1], GRF[2, :-1], label=r"F_z")
plt.legend(loc="upper left")
plt.xlabel(r"t [s]")
plt.ylabel('[N]')
plt.title("Ground reaction forces", fontdict=None, loc='center')
plt.grid()
if do_save_fig:
    plt.savefig(save_path+"GRF.pdf", format="pdf") # specify filetype explicitly

f2=plt.figure()
plt.plot(time_vector[:-1], q_p[0, :-1], label=r"$\mathrm{q}_{\mathrm{hip}}$")
plt.plot(time_vector[:-1], q_p[1, :-1], label=r"$\mathrm{q}_{\mathrm{knee}}$")
plt.legend(loc="upper left")
plt.xlabel(r"t [s]")
plt.ylabel(r"[rad]")
plt.title("Joint positions", fontdict=None, loc='center')
plt.grid()
if do_save_fig:
    plt.savefig(save_path+"q.pdf", format="pdf") # specify filetype explicitly

f3=plt.figure()
plt.plot(time_vector[:-1], q_p_dot[0, :-1], label=r"$\dot{q}_{\mathrm{hip}}$")
plt.plot(time_vector[:-1], q_p_dot[1, :-1], label=r"$\dot{q}_{\mathrm{knee}}$")
plt.legend(loc="upper left")
plt.xlabel(r"t [s]")
plt.ylabel(r"[rad/s]")
plt.title("Joint velocities", fontdict=None, loc='center')
plt.grid()
if do_save_fig:
    plt.savefig(save_path+"q_dot.pdf", format="pdf") # specify filetype explicitly

f4=plt.figure()
plt.plot(time_vector[:-1], q_p_ddot[0, :], label=r"$\ddot{\mathrm{q}}_{\mathrm{hip}}$")
plt.plot(time_vector[:-1], q_p_ddot[1, :], label=r"$\ddot{\mathrm{q}}_{\mathrm{knee}}$")
plt.legend(loc="upper left")
plt.xlabel(r"t [s]")
plt.ylabel(r"[$\mathrm{rad}/\mathrm{s}^2$]")
plt.title("Joint accelerations", fontdict=None, loc='center')
plt.grid()
if do_save_fig:
    plt.savefig(save_path+"q_ddot.pdf", format="pdf") # specify filetype explicitly

f5=plt.figure()
plt.plot(time_vector[:-1], tau[0, :],  drawstyle='steps-post', label=r"$\tau_{\mathrm{hip}}$ torque")
plt.plot(time_vector[:-1], tau[1, :],  drawstyle='steps-post', label=r"$\tau_{\mathrm{knee}}$ torque")
plt.legend(loc="upper left")
plt.xlabel(r"t [s]")
plt.ylabel(r"[Nm]")
plt.title("Joint efforts (link side)", fontdict=None, loc='center')
plt.grid()
if do_save_fig:
    plt.savefig(save_path+"link_side_efforts.pdf", format="pdf") # specify filetype explicitly
   
f6=plt.figure()
plt.plot(time_vector[:-1], i_q[0, :], label=r"$\mathrm{i}_\mathrm{q}^{\mathrm{hip}}$ current")
plt.plot(time_vector[:-1], i_q[1, :], label=r"$\mathrm{i}_\mathrm{q}^{\mathrm{knee}}$ current")
plt.legend(loc="upper left")
plt.xlabel(r"t [s]")
plt.ylabel(r"[A]")
plt.title("Actuators quadrature current", fontdict=None, loc='center')
plt.grid()
if do_save_fig:
    plt.savefig(save_path+"i_q.pdf", format="pdf") # specify filetype explicitly

f7=plt.figure()
plt.plot(time_vector[:-1], i_q[0, :]*hip_K_t, drawstyle='steps-post', label=r"$\tau_{\mathrm{hip}}$ torque")
plt.plot(time_vector[:-1], i_q[1, :]*knee_K_t, drawstyle='steps-post', label=r"$\tau_{\mathrm{knee}}$ torque")
plt.legend(loc="upper left")
plt.xlabel(r"t [s]")
plt.ylabel(r"[Nm]")
plt.title("Joint efforts (rotor side)", fontdict=None, loc='center')
plt.grid()
if do_save_fig:
    plt.savefig(save_path+"rotor_side_efforts.pdf", format="pdf") # specify filetype explicitly

f8=plt.figure()
plt.plot(time_vector[:-1], dt, drawstyle='steps-post')
plt.legend(loc="upper left")
plt.ylabel(r"t [s]")
plt.xlabel("node number")
plt.title("Optimization dt", fontdict=None, loc='center')
plt.grid()
if do_save_fig:
    plt.savefig(save_path+"opt_dt.pdf", format="pdf") # specify filetype explicitly

f9=plt.figure()
plt.plot(time_vector[:-1], P_hip,label=r"$P_{hip}$")
plt.plot(time_vector[:-1], P_knee,label=r"$P_{knee}$")
plt.legend(loc="upper left")
plt.xlabel(r"t [s]")
plt.ylabel(r"[W]")
plt.title("Mechanical power (actuator side)", fontdict=None, loc='center')
plt.grid()
if do_save_fig:
    plt.savefig(save_path+"mech_power.pdf", format="pdf") # specify filetype explicitly

f10=plt.figure()
plt.plot(time_vector, hip_height,label="hip excursion")
plt.plot(time_vector, foot_tip_height,label="foot tip excursion")
plt.legend(loc="upper left")
plt.xlabel(r"t [s]")
plt.ylabel(r"[m]")
plt.title("Characteristic link height excursion (w.r.t. their initial conditions)", fontdict=None, loc='center')
plt.grid()
if do_save_fig:
    plt.savefig(save_path+"link_heights.pdf", format="pdf") # specify filetype explicitly

plt.show()

