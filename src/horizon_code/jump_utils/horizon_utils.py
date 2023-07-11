from horizon.utils import kin_dyn
import numpy as np

def inv_dyn_from_sol(urdf_kin_dyn, q_p, q_p_dot, q_p_ddot, force_ref_frame, frame_force_map):

    ID = kin_dyn.InverseDynamics(urdf_kin_dyn, frame_force_map.keys(), force_ref_frame)

    n_intervals = q_p_ddot.shape[1]

    tau_res = np.zeros(q_p_ddot.shape)

    for i in range(n_intervals):
        frame_force_map_i = dict()
        for frame, wrench in frame_force_map.items():
            frame_force_map_i[frame] = wrench[:, i]
        tau_i = ID.call(q_p[:, i], q_p_dot[:, i], q_p_ddot[:, i], frame_force_map_i)
        tau_res[:, i] = tau_i.toarray().flatten()

    return tau_res

def forw_dyn_from_sol(urdf_kin_dyn, q_p, q_p_dot, tau, force_ref_frame, frame_force_map):
    
    FD = kin_dyn.ForwardDynamics(urdf_kin_dyn, frame_force_map.keys(), force_ref_frame)

    n_intervals = tau.shape[1]

    a_res = np.zeros(tau.shape)

    for i in range(n_intervals):
        frame_force_map_i = dict()
        for frame, wrench in frame_force_map.items():
            frame_force_map_i[frame] = wrench[:, i]
        a_i = FD.call(q_p[:, i], q_p_dot[:, i], tau[:, i], frame_force_map_i)
        a_res[:, i] = a_i.toarray().flatten()

    return a_res

def compute_required_iq_estimate(act_yaml_file, 
                                q_p_dot, 
                                q_p_ddot, 
                                tau, 
                                tanh_coeff):

    i_q_est = []
    n_jnts = len(act_yaml_file["K_d0"])
   
    for i in range(n_jnts):

        static_friction_effort = act_yaml_file["K_d0"][i] * np.tanh( tanh_coeff * q_p_dot[i + 1]) 
        dynamic_friction_effort = act_yaml_file["K_d1"][i] * q_p_dot[i + 1]
        link_side_effort = tau[i + 1]
        total_torque_on_motor = link_side_effort + static_friction_effort + dynamic_friction_effort
        actuator_red_ratio = act_yaml_file["red_ratio"][i]
        motor_omega_dot = q_p_ddot[i + 1] / actuator_red_ratio
        rotor_inertia = act_yaml_file["rotor_axial_MoI"][i]
        required_motor_torque = rotor_inertia * motor_omega_dot + total_torque_on_motor * actuator_red_ratio
        motor_torque_constant = act_yaml_file["K_t"][i]
        required_i_q_estimate = required_motor_torque / motor_torque_constant

        i_q_est.append(required_i_q_estimate)

    return i_q_est

def compute_required_iq_estimate_num(act_yaml_file, 
                                    q_p_dot_sol, 
                                    q_p_ddot_sol, 
                                    tau_sol, 
                                    tanh_coeff):

    # BROKEEEEEEENNNNNN -> NOT WORKING

    i_q_n_samples = len(q_p_ddot_sol[0, :])
    n_jnts = len(act_yaml_file["K_d0"])

    i_q_est = np.zeros((n_jnts, i_q_n_samples))

    for i in range(n_jnts):

        static_friction_effort = act_yaml_file["K_d0"][i] * np.tanh( tanh_coeff * q_p_dot_sol[i + 1, 1:(i_q_n_samples + 1)]) 
        dynamic_friction_effort = act_yaml_file["K_d1"][i] * q_p_dot_sol[i + 1, 1:(i_q_n_samples + 1)]
        link_side_effort = tau_sol[i + 1, :]
        total_torque_on_motor = link_side_effort + static_friction_effort + dynamic_friction_effort
        actuator_red_ratio = act_yaml_file["red_ratio"][i]
        motor_omega_dot = q_p_ddot_sol[i + 1, :] / actuator_red_ratio
        rotor_inertia = act_yaml_file["rotor_axial_MoI"][i]
        required_motor_torque = rotor_inertia * motor_omega_dot + total_torque_on_motor * actuator_red_ratio
        motor_torque_constant = act_yaml_file["K_t"][i]
        required_i_q_estimate = required_motor_torque / motor_torque_constant

        i_q_est[i, :] = required_i_q_estimate

    return i_q_est
