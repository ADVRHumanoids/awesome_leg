#!/usr/bin/env python3


import casadi as cs
import casadi_kin_dyn
import numpy as np
import rospy
from horizon.utils import mat_storer, resampler_trajectory, utils
import scipy.io 

import shutil

########################################################################################################################

urdf_path = rospy.get_param("/horizon/urdf_path")  # urdf relative path (wrt to the package)
dt_res = rospy.get_param("horizon/horizon_resampler/dt")
mat_path = rospy.get_param("horizon/horizon_resampler/mat_path")
dump_path = rospy.get_param("horizon/horizon_resampler/dump_path")

sol_mat_name = rospy.get_param("/horizon/horizon_solver/sol_mat_name")
res_sol_mat_name = rospy.get_param("/horizon/horizon_solver/res_sol_mat_name")

math_abs_path = mat_path + "/" + sol_mat_name + ".mat"

tanh_coeff = rospy.get_param("horizon/horizon_i_q_estimator/tanh_coeff")  # coefficient used by the approximated sign function ( sign = atanh(tanh_coeff * x) )

media_path = rospy.get_param("/horizon/media_path")  # urdf relative path (wrt to the package)
opt_res_path = rospy.get_param("/horizon/opt_results_path")  # urdf relative path (wrt to the package)

######################### Getting the properties of the actuator from the ROS parameter server #########################

actuators_param = rospy.get_param("/actuators")
actuator_names = list(actuators_param.keys())
n_actuators = len(actuator_names)

K_d0 = np.zeros((n_actuators, 1)).flatten()
K_d1 = np.zeros((n_actuators, 1)).flatten()
rotor_axial_MoI = np.zeros((n_actuators, 1)).flatten()
rotor_mass = np.zeros((n_actuators, 1)).flatten()
red_ratio = np.zeros((n_actuators, 1)).flatten()
efficiency = np.zeros((n_actuators, 1)).flatten()
K_t = np.zeros((n_actuators, 1)).flatten()
K_bmf = np.zeros((n_actuators, 1)).flatten()
R = np.zeros((n_actuators, 1)).flatten()
tau_max_br = np.zeros((n_actuators, 1)).flatten()
tau_max_ar = np.zeros((n_actuators, 1)).flatten()
tau_peak_br = np.zeros((n_actuators, 1)).flatten()
tau_peak_ar = np.zeros((n_actuators, 1)).flatten()
I_max  = np.zeros((n_actuators, 1)).flatten()
I_peak = np.zeros((n_actuators, 1)).flatten()
omega_max_nl_bf  = np.zeros((n_actuators, 1)).flatten()
omega_max_nl_af  = np.zeros((n_actuators, 1)).flatten()
omega_max_nl_af44  = np.zeros((n_actuators, 1)).flatten()
omega_max_nl_af112  = np.zeros((n_actuators, 1)).flatten()

for i in range(n_actuators):

    parameters = actuators_param[actuator_names[i]]

    K_d0[i] = parameters["K_d0"]
    K_d1[i] = parameters["K_d1"]
    rotor_axial_MoI[i] = parameters["rotor_axial_MoI"]
    rotor_mass[i] = parameters["rotor_mass"]
    red_ratio[i] = parameters["red_ratio"]
    efficiency[i] = parameters["efficiency"]
    K_t[i] = parameters["K_t"]
    K_bmf[i] = parameters["K_bmf"]
    R[i] = parameters["R"]
    tau_max_br[i] = parameters["tau_max_br"]
    tau_max_ar[i] = parameters["tau_max_ar"]
    tau_peak_br[i] = parameters["tau_peak_br"]
    tau_peak_ar[i] = parameters["tau_peak_ar"]
    I_max[i] = parameters["I_max"]
    I_peak[i] = parameters["I_peak"]
    omega_max_nl_bf[i] = parameters["omega_max_nl_bf"]
    omega_max_nl_af[i] = parameters["omega_max_nl_af"]
    omega_max_nl_af44[i] = parameters["omega_max_nl_af44"]
    omega_max_nl_af112[i] = parameters["omega_max_nl_af112"]

########################################################################################################################


sol = scipy.io.loadmat(math_abs_path)

urdf = open(urdf_path, "r").read()
urdf_awesome_leg = casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn(urdf)

n_q = urdf_awesome_leg.nq()  # number of joints
n_v = urdf_awesome_leg.nv()  # number of dofs

q_sym = cs.SX.sym('q', n_q)
q_dot_sym = cs.SX.sym('q_dot', n_v)
q_ddot_sym = cs.SX.sym('q_ddot', n_v)
x, x_dot = utils.double_integrator(q_sym, q_dot_sym, q_ddot_sym)

dae = {'x': x, 'p': q_ddot_sym, 'ode': x_dot, 'quad': 1}


########################################################################################################################

# hip
fk_hip = cs.Function.deserialize(urdf_awesome_leg.fk("hip1_1"))  # deserializing

# hip vel
dfk_hip = cs.Function.deserialize(
    urdf_awesome_leg.frameVelocity("hip1_1", casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))

# foot tip pos
fk_foot = cs.Function.deserialize(urdf_awesome_leg.fk("tip"))

# foot tip vel
dfk_foot = cs.Function.deserialize(
    urdf_awesome_leg.frameVelocity("tip", casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))


########################################################################################################################

sol_contact_map = dict(tip = sol["f_contact"])  # creating a contact map for applying the input to the foot

p_res, v_res, a_res, sol_contact_map_res, tau_res = resampler_trajectory.resample_torques(sol["q_p"],
                                                                                              sol["q_p_dot"],
                                                                                              sol["q_p_ddot"],
                                                                                              sol["dt_opt"].flatten(),
                                                                                              dt_res,
                                                                                              dae, sol_contact_map,
                                                                                              urdf_awesome_leg,
                                                                                              casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)

########################################################################################################################

foot_tip_position_init = fk_foot(q = p_res[:, 0])["ee_pos"]  
hip_position_initial = fk_hip(q = p_res[:, 0])["ee_pos"]  

########################################################################################################################


# Hip and knee quadrature current estimation
n_res_samples = len(p_res[0, :])
i_q_res = np.zeros((n_q, n_res_samples - 1))

for i in range(n_q - 1):
    compensated_tau = tau_res[i + 1, :] + K_d0[i] * np.tanh( tanh_coeff * v_res[i + 1, 1:(n_res_samples)]) + K_d1[i] * v_res[i + 1, 1:(n_res_samples)]
    i_q_res[i, :] = (rotor_axial_MoI[i] * a_res[i + 1, 0:(n_res_samples)] / red_ratio[i] + compensated_tau * red_ratio[i] / efficiency[i]) / K_t[i]

res_GRF = sol_contact_map_res["tip"]
res_q_p = p_res
res_foot_tip_position = fk_foot(q = p_res)["ee_pos"][2,:].toarray()  # foot position
res_hip_position = fk_hip(q=p_res)["ee_pos"][2,:].toarray()   # hip position
init_res_foot_tip_position_aux = np.tile(foot_tip_position_init,(1, n_res_samples )) # auxiliary matrix to compute position excursion
init_res_hip_position_aux = np.tile(hip_position_initial,(1, n_res_samples ))
res_v_foot_tip = dfk_foot(q=p_res, qdot=v_res)["ee_vel_linear"]  # foot velocity
res_v_foot_hip = dfk_hip(q=p_res, qdot=v_res)["ee_vel_linear"]  # foot velocity
dt_res_vector = np.tile(dt_res, n_res_samples - 1)

useful_solutions_res={"q_p":p_res,"q_p_dot":v_res, "q_p_ddot":a_res,
                 "tau":tau_res, "f_contact":res_GRF, "i_q":i_q_res, "dt_opt":dt_res_vector,
                 "foot_tip_height":np.transpose(res_foot_tip_position-init_res_foot_tip_position_aux[2,:]), 
                 "hip_height":np.transpose(res_hip_position-init_res_hip_position_aux[2,:]), 
                 "tip_velocity":np.transpose(np.transpose(res_v_foot_tip)),
                 "hip_velocity":np.transpose(np.transpose(res_v_foot_hip))}

ms_res = mat_storer.matStorer(opt_res_path + "/jump_test/horizon_offline_solver_res.mat") # resampled sol
ms_res.store(useful_solutions_res) # saving solution data to file

# copying stuff for future debugging
shutil.copyfile(opt_res_path + "/jump_test/horizon_offline_solver_res.mat", dump_path + "/" + res_sol_mat_name + ".mat")
 
