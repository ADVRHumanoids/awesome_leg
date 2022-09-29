#!/usr/bin/env python3

import yaml

import argparse

import os as scibidibi
import shutil
import time

import casadi as cs
from sympy import N
import casadi_kin_dyn
import numpy as np
import rospkg
import rospy
from horizon.problem import Problem
from horizon.ros.replay_trajectory import *
from horizon.solvers import solver
from horizon.transcriptions import transcriptor
from horizon.utils import kin_dyn, mat_storer, resampler_trajectory, utils

from jump_utils.miscell_utils import str2bool

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(
        description='Pipeline script for the generation of offline jumps of the Awesome Leg')

    # first level specific arguments
    parser.add_argument('--urdf_path', '-urdf', type = str)
    parser.add_argument('--unique_id', '-id', type = str)
    parser.add_argument('-yaml_path', "-yaml", type = str)
    parser.add_argument('-actuators_yaml_path', "-ayaml", type = str)
    parser.add_argument('-sol_mat_name', "-matn", type = str)
    parser.add_argument('-sol_mat_name_res', "-matn_res", type = str)
    parser.add_argument('--results_dir', '-rdir', type = str)

    parser.add_argument('--is_cyclic_jump', '-cjump', default= False, type = bool)

    parser.add_argument('--is_cal', '-cal', default= False, type = bool)

    args = parser.parse_args()

    yaml_file = None
    with open(args.yaml_path, "r") as stream:
        try:
            yaml_file = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    act_yaml_file = None
    with open(args.actuators_yaml_path, "r") as stream:
        try:
            act_yaml_file = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    is_refine_phase = False

    tanh_coeff = yaml_file["i_q_estimation"]["tanh_coeff"]

    n_int = yaml_file["problem"]["n_int"]
    n_takeoff = yaml_file["problem"]["n_takeoff"]
    n_touchdown = yaml_file["problem"]["n_touchdown"]

    dt_lb = yaml_file["problem"]["dt_lb"]
    dt_ub = yaml_file["problem"]["dt_ub"]

    jnt_limit_margin = yaml_file["problem"]["jnt_limit_margin"]
    jnt_vel_limit_margin = yaml_file["problem"]["jnt_vel_limit_margin"]

    slvr_opt = {"ipopt.tol": yaml_file["solver"]["ipopt_tol"],
        "ipopt.max_iter": yaml_file["solver"]["ipopt_maxiter"],
        "ipopt.constr_viol_tol": yaml_file["solver"]["ipopt_cnstr_viol_tol"],
        "ipopt.linear_solver": yaml_file["solver"]["ipopt_lin_solver"]
    }

    slvr_name = yaml_file["solver"]["name"] 

    trans_name = yaml_file["transcription"]["name"] 
    trans_integrator = yaml_file["transcription"]["integrator_name"] 

    sol_mat_name = args.sol_mat_name
    res_sol_mat_name = args.sol_mat_name_res

    ms_orig = mat_storer.matStorer(args.results_dir + "/" + sol_mat_name + ".mat")  # original opt. sol
    ms_resampl = mat_storer.matStorer(args.results_dir + "/" + res_sol_mat_name + ".mat")  # original opt. sol

    ## Actuator properties (see actuators.yaml for a brief description of most of the parameters)

    actuator_names = list(act_yaml_file.keys())
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

        parameters = act_yaml_file[actuator_names[i]]

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

        
        I_lim = I_peak # limits set to the peak current 

        ## setting up problem 

        urdf = open(args.urdf_path, "r").read()
        urdf_awesome_leg = casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn(urdf)

        n_q = urdf_awesome_leg.nq()  # number of joints
        n_v = urdf_awesome_leg.nv()  # number of dofs

        jnt_lim_margin_array = np.tile(jnt_limit_margin, (n_q))
        v_bounds = np.array(omega_max_nl_af44)
        v_bounds = v_bounds - v_bounds * jnt_vel_limit_margin

        lbs = urdf_awesome_leg.q_min() + jnt_lim_margin_array
        ubs = urdf_awesome_leg.q_max() - jnt_lim_margin_array

        tau_lim = np.array([0, tau_peak_ar[0], tau_peak_ar[1]])  # effort limits (0 on the passive d.o.f.)

        # tau_lim = np.array([0, cs.inf, cs.inf])  # effort limits (0 on the passive d.o.f.)

        prb = Problem(n_int)  # initialization of a problem object

        transcriptor_name = trans_name  
        trans_opt = dict(integrator = trans_integrator)  # dictionary with the chosen integrator name

        dt1 = prb.createSingleVariable("dt1", 1)  # dt before the takeoff
        dt2 = prb.createSingleVariable("dt2", 1)  # dt during flight
        dt3 = prb.createSingleVariable("dt3", 1)  # dt after touchdown
        dt1.setBounds(dt_lb, dt_ub)  # bounds on dt1
        dt2.setBounds(dt_lb, dt_ub)  # bounds on dt2
        dt3.setBounds(dt_lb, dt_ub)  # bounds on dt3

        dt=[dt1]* (n_takeoff) + [dt2] * (n_touchdown - n_takeoff) +\
            [dt3] * (n_int - n_touchdown) # holds the complete time list

        prb.setDt(dt)

        # Creating the state variables
        q_p = prb.createStateVariable("q_p", n_q)
        q_p_dot = prb.createStateVariable("q_p_dot",
                                        n_v)  # here q_p_dot is actually not the derivative of the lagrangian state vector
        q_p.setBounds(lbs, ubs) 
        q_p_dot[1:3].setBounds(- v_bounds, v_bounds)

        # Defining the input/s (joint accelerations)
        q_p_ddot = prb.createInputVariable("q_p_ddot", n_v)  # using joint accelerations as an input variable

        x, xdot = utils.double_integrator(q_p, q_p_dot, q_p_ddot)  # building the full state

        # Creating an additional input variable for the contact forces on the foot tip
        f_contact = prb.createInputVariable("f_contact", 3)  # dimension 3
        
        f_contact[2].setInitialGuess(urdf_awesome_leg.mass() * 9.81)

        f_contact[2].setLowerBounds(0)  # tip cannot be pulled from the ground

        contact_map = dict(tip1 = f_contact)  # creating a contact map for applying the input to the foot

        prb.setDynamics(xdot)  # setting the dynamics we are interested of in the problem object (xdot)

        trscptr = transcriptor.Transcriptor.make_method(transcriptor_name, prb, trans_opt)  # setting the transcriptor

        tau = kin_dyn.InverseDynamics(urdf_awesome_leg, contact_map.keys(), \
                casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED).call(q_p, q_p_dot, q_p_ddot, contact_map) 

        # hip
        fk_hip = cs.Function.deserialize(urdf_awesome_leg.fk("base_link"))  # deserializing
        hip_position = fk_hip(q = q_p)["ee_pos"]  # hip position (symbolic)

        # hip vel
        dfk_hip = cs.Function.deserialize(
            urdf_awesome_leg.frameVelocity("base_link",\
                    casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
        v_hip = dfk_hip(q = q_p, qdot = q_p_dot)["ee_vel_linear"]  # foot velocity

        # foot tip pos
        fk_foot = cs.Function.deserialize(urdf_awesome_leg.fk("tip1"))
        foot_tip_position = fk_foot(q = q_p)["ee_pos"]  # foot position

        # foot tip vel
        dfk_foot = cs.Function.deserialize(
            urdf_awesome_leg.frameVelocity("tip1",\
                casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
        v_foot_tip = dfk_foot(q=q_p, qdot=q_p_dot)["ee_vel_linear"]  # foot velocity

        # center of mass
        com_f = cs.Function.deserialize(urdf_awesome_leg.centerOfMass())
        com = com_f(q = q_p, v = q_p_dot, a = q_p_ddot)["com"]

        # constraints
        tau_limits = prb.createIntermediateConstraint("tau_limits", tau)  # torque limits
        tau_limits.setBounds(-tau_lim, tau_lim)  # setting input limits

        pre_takeoff_nodes = range(0, n_takeoff + 1)
        post_touchdown_nodes = range(n_touchdown, n_int + 1)
        flight_nodes = range(n_takeoff + 1, n_touchdown)
        prb.createConstraint("foot_vel_bf_touchdown", v_foot_tip,
                            nodes=pre_takeoff_nodes)  # no velocity of the foot before takeoff
        prb.createConstraint("foot_vel_aftr_touchdown", v_foot_tip,
                            nodes=post_touchdown_nodes)  # no velocity of the foot after touchdown

        tip_above_ground = prb.createConstraint("tip_above_ground", foot_tip_position[2], nodes=flight_nodes)  # no ground penetration on all the horizoin
        tip_above_ground.setBounds(0.0, cs.inf)

        prb.createConstraint("GRF_zero", f_contact,
                            nodes=flight_nodes)  # 0 GRF during flight
        
        if args.is_cyclic_jump:

            prb.createFinalConstraint("leg_pose_restoration", q_p - q_p_ddot.getVarOffset(- (n_int + 1)))

        prb.createFinalConstraint("final_joint_zero_vel", q_p_dot)  # joints are still at the end of the optimization horizon

        # Keep the ESTIMATED (with the calibrated current model) quadrature currents within bounds
        is_iq_cnstrnt = rospy.get_param("horizon/horizon_solver/problem_settings/is_iq_cnstrnt")

        if is_iq_cnstrnt and is_refine_phase: # highly non linear constraint due to approximate sign function--> makes optimization much worse
            # use it only in the refinement phase, since it is expensive
            compensated_tau = []
            i_q_cnstr = []
            for i in range(n_q -1):
                compensated_tau.append(tau[i + 1] + K_d0[i] * np.tanh( tanh_coeff * q_p_dot[i + 1]) + K_d1[i] * q_p_dot[i + 1])
                i_q = (rotor_axial_MoI[i] * q_p_ddot[i + 1] / red_ratio[i] + compensated_tau[i] * red_ratio[i] / efficiency[i]) / K_t[i]
                i_q_cnstr.append( prb.createIntermediateConstraint("quadrature_current" + actuator_names[i], i_q) )
                i_q_cnstr[i].setBounds(-I_peak[i], I_peak[i])  # setting input limits

        # friction constraints
        is_friction_cone = rospy.get_param("horizon/horizon_solver/problem_settings/is_friction_cone")

        mu_friction_cone = abs(rospy.get_param("horizon/horizon_solver/problem_settings/mu_friction_cone"))

        if is_friction_cone:

            # exact friction cone (heavier to evaluate)
            # friction_cone_bf_takeoff = prb.createConstraint("friction_cone_bf_takeoff",\
            #                                     (f_contact[0]**2 + f_contact[1]**2) - (mu_friction_cone * f_contact[2])**2,
            #                      nodes=range(0, n_takeoff))
            # friction_cone_bf_takeoff.setBounds(-cs.inf, 0)

            # friction_cone_aftr_touchdown = prb.createConstraint("friction_cone_aftr_touchdown",\
            #                                     (f_contact[0]**2 + f_contact[1]**2) - (mu_friction_cone * f_contact[2])**2,
            #                      nodes=range(n_touchdown, n_int)) # f_contact not active on last node
            # friction_cone_aftr_touchdown.setBounds(-cs.inf, 0)

            # linearized friction cone (constraint need to be split in two because setBounds only takes numerical vals)
            friction_cone_bf_takeoff1 = prb.createConstraint("friction_cone_bf_takeoff1",\
                                                f_contact[1] - (mu_friction_cone * f_contact[2]),
                                nodes=pre_takeoff_nodes)
            friction_cone_bf_takeoff1.setBounds(-cs.inf, 0)
            friction_cone_bf_takeoff2 = prb.createConstraint("friction_cone_bf_takeoff2",\
                                                f_contact[1] + (mu_friction_cone * f_contact[2]),
                                nodes=pre_takeoff_nodes)
            friction_cone_bf_takeoff2.setBounds(0, cs.inf)

            friction_cone_aftr_touchdown1 = prb.createConstraint("friction_cone_aftr_touchdown1",\
                                                f_contact[1] - mu_friction_cone * f_contact[2],
                                nodes=post_touchdown_nodes[:-1]) # f_contact not active on last node
            friction_cone_aftr_touchdown1.setBounds(-cs.inf, 0)
            friction_cone_aftr_touchdown2 = prb.createConstraint("friction_cone_aftr_touchdown2",\
                                                f_contact[1] + mu_friction_cone * f_contact[2],
                                nodes=post_touchdown_nodes[:-1]) # f_contact not active on last node
            friction_cone_aftr_touchdown2.setBounds(0, cs.inf)

        ## costs
        epsi = 1

        scale_factor_base = yaml_file["problem"]["weights"]["scale_factor_costs_base"]  

        cost_scaling_factor = dt_lb * n_int * scale_factor_base

        weight_f_contact_diff = yaml_file["problem"]["weights"]["weight_f_contact_diff"]  
        weight_f_contact_cost = yaml_file["problem"]["weights"]["weight_f_contact"] 
        weight_q_ddot = yaml_file["problem"]["weights"]["weight_q_p_ddot"] 
        weight_input_diff = yaml_file["problem"]["weights"]["weight_input_diff"] 

        weight_com_height = yaml_file["problem"]["weights"]["weight_com_height"] 
        weight_hip_height = yaml_file["problem"]["weights"]["weight_hip_height"] 
        weight_tip_clearance = yaml_file["problem"]["weights"]["weight_tip_clearance"] 
        

        # common costs

        weight_contact_cost = weight_contact_cost / cost_scaling_factor
        prb.createIntermediateCost("min_f_contact", weight_contact_cost * cs.sumsqr(f_contact[0:2]))

        weight_q_ddot = weight_q_ddot / cost_scaling_factor
        prb.createIntermediateCost("min_q_ddot", weight_q_ddot * cs.sumsqr(
            q_p_ddot[1:]))  # minimizing the joint accelerations ("responsiveness" of the trajectory)

        weight_hip_height_jump = weight_hip_height_jump / cost_scaling_factor
        prb.createIntermediateCost("max_hip_height_jump", weight_hip_height_jump * cs.sumsqr(1 / (hip_position[2] + epsi)),
                                nodes=flight_nodes)

        weight_tip_clearance = weight_tip_clearance / cost_scaling_factor
        prb.createIntermediateCost("max_foot_tip_clearance", weight_tip_clearance * cs.sumsqr(1 / (foot_tip_position[2] + epsi)),
                                nodes=flight_nodes)

        weight_com_height = weight_com_height / cost_scaling_factor
        prb.createIntermediateCost("max_com_height", weight_com_height * cs.sumsqr(1/ ( com[2] + epsi)), 
                                nodes=flight_nodes)

        weight_min_input_diff = weight_min_input_diff / cost_scaling_factor
        prb.createIntermediateCost("min_input_diff", weight_min_input_diff * cs.sumsqr(q_p_ddot[1:] - q_p_ddot[1:].getVarOffset(-1)), 
                                    nodes = range(1, n_int))  

        weight_min_f_contact_diff = weight_min_f_contact_diff / cost_scaling_factor
        prb.createIntermediateCost("min_f_contact_diff", weight_min_input_diff * cs.sumsqr(f_contact - f_contact.getVarOffset(-1)), 
                                    nodes = range(1, n_int)) # doesn't seem to affect the results --> not useful

        ## solving

        slvr = solver.Solver.make_solver(slvr_name, prb, slvr_opt) 
        t = time.time()
        slvr.solve()  # solving
        solution_time = time.time() - t
        print(f'solved in {solution_time} s')
        solution = slvr.getSolutionDict() # extracting solution
        cnstr_opt = slvr.getConstraintSolutionDict()
        tau_sol = cnstr_opt["tau_limits"]

        q_sym = cs.SX.sym('q', n_q)
        q_dot_sym = cs.SX.sym('q_dot', n_v)
        q_ddot_sym = cs.SX.sym('q_ddot', n_v)
        x, x_dot = utils.double_integrator(q_sym, q_dot_sym, q_ddot_sym)

        dae = {'x': x, 'p': q_ddot_sym, 'ode': x_dot, 'quad': 1}

        dt_res = yaml_file["resampling"]["dt"]

        sol_contact_map = dict(tip1 = solution["f_contact"])  # creating a contact map for applying the input to the foot

        p_res, v_res, a_res, sol_contact_map_res, tau_res = resampler_trajectory.resample_torques(solution["q_p"],
                                                                solution["q_p_dot"],
                                                                solution["q_p_ddot"],
                                                                slvr.getDt().flatten(),
                                                                dt_res,
                                                                dae, sol_contact_map,
                                                                urdf_awesome_leg,
                                                                casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)

        # post-processing original solution

        # Hip and knee quadrature current estimation
        i_q_n_samples = len(solution["q_p_ddot"][0, :])
        i_q = np.zeros((n_q - 1, i_q_n_samples))
        for i in range(n_q - 1):
            compensated_tau = tau_sol[i + 1, :] + K_d0[i] * np.tanh( tanh_coeff * solution["q_p_dot"][i + 1, 1:(i_q_n_samples + 1)]) + \
                                                K_d1[i] * solution["q_p_dot"][i + 1, 1:(i_q_n_samples + 1)]
            i_q[i, :] = (rotor_axial_MoI[i] * solution["q_p_ddot"][i + 1, :] / red_ratio[i] +\
                        compensated_tau * red_ratio[i] / efficiency[i]) / K_t[i]

        solution_GRF = solution["f_contact"]
        solution_q_p = solution["q_p"]
        solution_foot_tip_position = fk_foot(q = solution_q_p)["ee_pos"][2,:].toarray()  # foot position
        solution_hip_position = fk_hip(q=solution_q_p)["ee_pos"][2,:].toarray()   # hip position
        solution_v_foot_tip = dfk_foot(q=solution["q_p"], qdot=solution["q_p_dot"])["ee_vel_linear"]  # foot velocity
        solution_v_foot_hip = dfk_hip(q=solution["q_p"], qdot=solution["q_p_dot"])["ee_vel_linear"]  # foot velocity

        other_stuff = {"tau":cnstr_opt["tau_limits"], "i_q":i_q, "dt_opt":slvr.getDt(),
                    "foot_tip_height": np.transpose(solution_foot_tip_position), 
                    "hip_height": np.transpose(solution_hip_position), 
                    "tip_velocity": np.transpose(np.transpose(solution_v_foot_tip)),
                    "hip_velocity": np.transpose(np.transpose(solution_v_foot_hip)),
                    "sol_time": solution_time,
                    "is_calibrated": args.is_cal, 
                    "weight_min_input_diff": weight_input_diff, 
                    "weight_min_f_contact_diff": weight_f_contact_diff, 
                    "is_refine_phase": is_refine_phase, 
                    "unique_id": args.unique_id}

        sol_dict_full = {}

        # Hip and knee quadrature current estimation
        n_res_samples = len(p_res[0, :])
        i_q_res = np.zeros((n_q - 1, n_res_samples - 1))

        for i in range(n_q - 1):
            compensated_tau = tau_res[i + 1, :] + K_d0[i] * np.tanh( tanh_coeff * v_res[i + 1, 1:(n_res_samples)]) +\
                                                K_d1[i] * v_res[i + 1, 1:(n_res_samples)]
            i_q_res[i, :] = (rotor_axial_MoI[i] * a_res[i + 1, 0:(n_res_samples)] / red_ratio[i] + \
                            compensated_tau * red_ratio[i] / efficiency[i]) / K_t[i]

        res_GRF = sol_contact_map_res["tip1"]
        res_q_p = p_res
        res_foot_tip_position = fk_foot(q = p_res)["ee_pos"][2,:].toarray()  # foot position
        res_hip_position = fk_hip(q=p_res)["ee_pos"][2,:].toarray()   # hip position
        res_v_foot_tip = dfk_foot(q=p_res, qdot=v_res)["ee_vel_linear"]  # foot velocity
        res_v_foot_hip = dfk_hip(q=p_res, qdot=v_res)["ee_vel_linear"]  # foot velocity
        dt_res_vector = np.tile(dt_res, n_res_samples - 1)

        useful_solutions_res={"q_p":p_res,"q_p_dot":v_res, "q_p_ddot":a_res,
                        "tau":tau_res, "f_contact":res_GRF, "i_q":i_q_res, "dt_opt":dt_res_vector,
                        "foot_tip_height":np.transpose(res_foot_tip_position), 
                        "hip_height":np.transpose(res_hip_position), 
                        "tip_velocity":np.transpose(np.transpose(res_v_foot_tip)),
                        "hip_velocity":np.transpose(np.transpose(res_v_foot_hip))}

        sol_dict_full_resampl= {**useful_solutions_res}

        sol_dict_full_orig = {**solution,
                **cnstr_opt,
                **other_stuff}

        ms_orig.store(sol_dict_full_orig) # saving solution data to file

        ms_resampl.store(sol_dict_full_resampl) # saving solution data to file




