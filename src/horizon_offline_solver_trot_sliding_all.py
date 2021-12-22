#!/usr/bin/env python3

##################### Imports #########################
import time

import rospy
import rospkg

from horizon.problem import Problem
from horizon.utils import kin_dyn, utils, mat_storer,resampler_trajectory
from horizon.transcriptions import transcriptor
from horizon.solvers import solver
import casadi as cs
import casadi_kin_dyn

import numpy as np

import os as scibidibi 
import shutil
from datetime import date

from horizon.ros.replay_trajectory import *

##################### Reading the solution modes from config YAML config file #########################

save_sol_as_init = rospy.get_param("horizon/horizon_solver/save_sol_as_init")  # if true, the solution is also saved as a candidate for future optimization initializations
employ_opt_init = rospy.get_param("horizon/horizon_solver/employ_opt_init")  # if true, the solution is also saved as a candidate for future optimization initializations

urdf_rel_path = rospy.get_param("/horizon/urdf_relative_path")  # urdf relative path (wrt to the package)

media_rel_path = rospy.get_param("/horizon/media_relative_path")  # urdf relative path (wrt to the package)
opt_res_rel_path = rospy.get_param("/horizon/opt_results_rel_path")  # urdf relative path (wrt to the package)

##################### Initializing objects for .mat storage #########################

## Getting/setting some useful variables
rospackage=rospkg.RosPack() # Only for taking the path to the leg package
today = date.today()
today_is = today.strftime("%d-%m-%Y")
config_path=rospackage.get_path("awesome_leg_pholus")+"/config/" # configuration files path

## Creating folders for saving plots and other data (if not already existing). This folders are also used by horizon_plot.py

if  (not scibidibi.path.isdir(rospackage.get_path("awesome_leg_pholus")+media_rel_path+"/"+today_is)):
    scibidibi.makedirs(rospackage.get_path("awesome_leg_pholus")+media_rel_path+"/"+today_is)

ms = mat_storer.matStorer(rospackage.get_path("awesome_leg_pholus")+opt_res_rel_path+"/horizon_offline_solver.mat")
ms_aux = mat_storer.matStorer(rospackage.get_path("awesome_leg_pholus")+media_rel_path+"/"+today_is+"/horizon_offline_solver.mat")
target=rospackage.get_path("awesome_leg_pholus")+media_rel_path+"/"+today_is

shutil.copyfile(config_path+"actuators.yaml", target+"/actuators.yaml") # saving config files for reference and future debugging
shutil.copyfile(config_path+"horizon_trot_sliding_all.yaml", target+"/horizon.yaml")
shutil.copyfile(config_path+"xbot2.yaml", target+"/xbot2.yaml") 

if save_sol_as_init: # save the solution as the initialization for the next sim
    ms_opt_init = mat_storer.matStorer(rospackage.get_path("awesome_leg_pholus")+opt_res_rel_path+"/horizon_offline_solver_init.mat")

if employ_opt_init: # initialize variables with the previously saved solution
    ms_load_path=rospackage.get_path("awesome_leg_pholus")+opt_res_rel_path+"/horizon_offline_solver_init.mat"
    ms_load = mat_storer.matStorer(ms_load_path)
    shutil.copyfile(ms_load_path, target) # copying used init to folder for reference and debugging
    loaded_sol=ms_load.load() # loading the solution dictionary

##################### LOADING SOLVER PARAMETERS FROM SERVER #########################

## Problem parameters, based on the chosen options

n_nodes = rospy.get_param("horizon/horizon_solver/variable_dt/problem_settings/n_nodes")  # optimization horizon
n_takeoff =rospy.get_param("horizon/horizon_solver/variable_dt/problem_settings/n_takeoff")   # instant of takeoff
dt= rospy.get_param("horizon/horizon_solver/variable_dt/problem_settings/dt")   # dt

forward_vel= rospy.get_param("horizon/horizon_solver/variable_dt/problem_settings/forward_vel") 

tip_ground_clearance= rospy.get_param("horizon/horizon_solver/variable_dt/problem_settings/tip_ground_clearance") 

dt_lb=rospy.get_param("horizon/horizon_solver/variable_dt/problem_settings/dt_lb")   # dt lower bound 
dt_ub=rospy.get_param("horizon/horizon_solver/variable_dt/problem_settings/dt_ub")   # dt upper bound
test_rig_lb= rospy.get_param("horizon/horizon_solver/variable_dt/problem_settings/test_rig/lb") # lower bound of the test rig excursion
test_rig_ub=rospy.get_param("horizon/horizon_solver/variable_dt/problem_settings/test_rig/ub") # upper bound of the test rig excursion

# cost weights

weight_contact_cost = rospy.get_param("horizon/horizon_solver/variable_dt/problem_settings/cost_weights/contact_force")  # minimizing the contact force
weight_fictitious_actuation= rospy.get_param("horizon/horizon_solver/variable_dt/problem_settings/cost_weights/weight_fictitious_actuation") # minimizing the fictitious actuations
# weight_postural_cost = rospy.get_param("horizon/horizon_solver/variable_dt/problem_settings/cost_weights/terminal_postural") # cost to restore the initial position at the end of the control horizon
weight_q_ddot = rospy.get_param("horizon/horizon_solver/variable_dt/problem_settings/cost_weights/small_q_p_ddot")# minimizing joint accelerations
# weight_hip_i_d=rospy.get_param("horizon/horizon_solver/variable_dt/problem_settings/cost_weights/weight_hip_i_d") # minimizing hip i_d
# weight_knee_i_d=rospy.get_param("horizon/horizon_solver/variable_dt/problem_settings/cost_weights/weight_knee_i_d") # minimizing hip i_d
weight_forward_vel=rospy.get_param("horizon/horizon_solver/variable_dt/problem_settings/cost_weights/weight_forward_vel") # maximizing the jump height (measured at the tip)
weight_large_tip_vert_excursion=rospy.get_param("horizon/horizon_solver/variable_dt/problem_settings/cost_weights/weight_forward_vel") 
# solver options

slvr_opt = {"ipopt.tol": rospy.get_param("horizon/horizon_solver/variable_dt/problem_settings/solver/tolerance"), 
"ipopt.max_iter": rospy.get_param("horizon/horizon_solver/variable_dt/problem_settings/solver/max_iter"), 
"ipopt.linear_solver": rospy.get_param("horizon/horizon_solver/variable_dt/problem_settings/solver/linear_solver_name")} 
slvr_name=rospy.get_param("horizon/horizon_solver/variable_dt/problem_settings/solver/name") # options: "blocksqp", "ipopt", "ilqr", "gnsqp", 

################# Getting the properties of the actuator from the ROS parameter server #########################

## hip actuator:
hip_rotor_axial_MoI=rospy.get_param("/actuators/hip/rotor_axial_MoI")
hip_rotor_mass=rospy.get_param("/actuators/hip/rotor_mass")
hip_red_ratio=rospy.get_param("/actuators/hip/red_ratio")
hip_efficiency=rospy.get_param("/actuators/hip/efficiency")
hip_K_t=rospy.get_param("/actuators/hip/K_t")
hip_K_bmf=rospy.get_param("/actuators/hip/K_bmf")
hip_R=rospy.get_param("/actuators/hip/R")
hip_tau_max_br=rospy.get_param("/actuators/hip/tau_max_br")
hip_tau_max_ar=rospy.get_param("/actuators/hip/tau_max_ar")
hip_tau_peak_br=rospy.get_param("/actuators/hip/tau_peak_br")
hip_tau_peak_ar=rospy.get_param("/actuators/hip/tau_peak_ar")
hip_I_max=rospy.get_param("/actuators/hip/I_max")
hip_I_peak=rospy.get_param("/actuators/hip/I_peak")

hip_omega_max_nl_bf=rospy.get_param("/actuators/hip/omega_max_nl_bf")
hip_omega_max_nl_af=rospy.get_param("/actuators/hip/omega_max_nl_af")
hip_omega_max_nl_af44=rospy.get_param("/actuators/hip/omega_max_nl_af44")
hip_omega_max_nl_af112=rospy.get_param("/actuators/hip/omega_max_nl_af112")

## knee actuator:
knee_rotor_axial_MoI=rospy.get_param("/actuators/knee/rotor_axial_MoI")
knee_rotor_mass=rospy.get_param("/actuators/knee/rotor_mass")
knee_red_ratio=rospy.get_param("/actuators/knee/red_ratio")
knee_efficiency=rospy.get_param("/actuators/knee/efficiency")
knee_K_t=rospy.get_param("/actuators/knee/K_t")
knee_K_bmf=rospy.get_param("/actuators/knee/K_bmf")
knee_R=rospy.get_param("/actuators/knee/R")
knee_tau_max_br=rospy.get_param("/actuators/knee/tau_max_br")
knee_tau_max_ar=rospy.get_param("/actuators/knee/tau_max_ar")
knee_tau_peak_br=rospy.get_param("/actuators/knee/tau_peak_br")
knee_tau_peak_ar=rospy.get_param("/actuators/knee/tau_peak_ar")
knee_I_max=rospy.get_param("/actuators/knee/I_max")
knee_I_peak=rospy.get_param("/actuators/knee/I_peak")
knee_omega_max_nl_bf=rospy.get_param("/actuators/knee/omega_max_nl_bf")
knee_omega_max_nl_af=rospy.get_param("/actuators/knee/omega_max_nl_af")
knee_omega_max_nl_af44=rospy.get_param("/actuators/knee/omega_max_nl_af44")
knee_omega_max_nl_af112=rospy.get_param("/actuators/knee/omega_max_nl_af112")

#################### Loading the URDF ##########################

urdf_path = rospackage.get_path("awesome_leg_pholus")+urdf_rel_path
urdf = open(urdf_path, "r").read()
urdf_awesome_leg = casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn(urdf)

n_q = urdf_awesome_leg.nq()  # number of joints
n_v = urdf_awesome_leg.nv()  # number of dofs

##############################################

tau_lim = np.array([cs.inf, cs.inf, cs.inf, cs.inf])  # effort limits (excluding the fictious linear actuations on the hip joint)

prb = Problem(n_nodes)  # initialization of a problem object

transcriptor_name = "multiple_shooting"  # other option: "direct_collocation"
trans_opt = dict(integrator="RK4")  # dictionary with the chosen integrator name

##############################################

dt1 = prb.createSingleVariable("dt1", 1)  # dt for the foot tip topuch phase
dt2 = prb.createSingleVariable("dt2", 1)  # dt for the flight phase
dt1.setBounds(dt_lb, dt_ub)  # bounds on dt1
dt2.setBounds(dt_lb, dt_ub)  # bounds on dt2

Dt=[dt1]*n_takeoff+[dt2]*(n_nodes-n_takeoff) # holds the complete time list

# prb.setDt(Dt)
prb.setDt(dt)

# Creating the state variables
q_p = prb.createStateVariable("q_p", n_q)
if employ_opt_init:
    q_p[1:].setInitialGuess(loaded_sol["q_p"])

q_p_dot = prb.createStateVariable("q_p_dot",
                                  n_v)  # here q_p_dot is actually not the derivative of the lagrangian state vector
if employ_opt_init:
    q_p_dot[1:].setInitialGuess(loaded_sol["q_p_dot"])

q_p[1].setBounds(test_rig_lb, test_rig_ub) # test rig excursion

# Defining the input/s (joint accelerations)
q_p_ddot = prb.createInputVariable("q_p_ddot", n_v)  # using joint accelerations as an input variable
if employ_opt_init:
    q_p_ddot[1:].setInitialGuess(loaded_sol["q_p_ddot"])

x, xdot = utils.double_integrator(q_p, q_p_dot, q_p_ddot)  # building the full state

# Creating an additional input variable for the contact forces on the foot tip
f_contact = prb.createInputVariable("f_contact", 3)  # dimension 3
if employ_opt_init:
    f_contact.setInitialGuess(loaded_sol["f_contact"])
else:
    f_contact[2].setInitialGuess(100.0)

f_contact[2].setLowerBounds(0)  # the vertical component of f_contact needs to be always positive
contact_map = dict(tip=f_contact)  # creating a contact map for applying the input to the foot

# 
q_p_init = prb.createSingleVariable("q_p_init", n_q)  # single variable for leaving the initial condition free
q_p_dot_init = prb.createSingleVariable("q_p_dot_init", n_q)  # single variable for leaving the initial condition free

##############################################

prb.setDynamics(xdot)  # setting the dynamics we are interested of in the problem object (xdot)
trscptr = transcriptor.Transcriptor.make_method(transcriptor_name, prb, trans_opt)  # setting the transcriptor

##############################################

## Obtaining some relevant quantities
tau = kin_dyn.InverseDynamics(urdf_awesome_leg, contact_map.keys(),casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED).call(q_p, q_p_dot,q_p_ddot,contact_map) # obtaining the joint efforts

# hip pos
fk_hip = cs.Function.deserialize(urdf_awesome_leg.fk("hip1_1"))  # deserializing
hip_position_initial = fk_hip(q=q_p_init)["ee_pos"]  # initial hip position (numerical)
hip_position = fk_hip(q=q_p)["ee_pos"]  # hip position (symbolic)

# hip vel
dfk_hip = cs.Function.deserialize(
    urdf_awesome_leg.frameVelocity("hip1_1", casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
v_hip = dfk_hip(q=q_p, qdot=q_p_dot)["ee_vel_linear"]  # foot velocity

# foot tip pos
fk_foot = cs.Function.deserialize(urdf_awesome_leg.fk("tip"))
foot_tip_position_init = fk_foot(q=q_p_init)["ee_pos"]  # foot initial position
foot_tip_position = fk_foot(q=q_p)["ee_pos"]  # foot position

# foot tip vel
dfk_foot = cs.Function.deserialize(
    urdf_awesome_leg.frameVelocity("tip", casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
v_foot_tip = dfk_foot(q=q_p, qdot=q_p_dot)["ee_vel_linear"]  # foot velocity

##############################################

## Constraints
tau_cnstrnt = prb.createIntermediateConstraint("dynamics_feas", tau[0])  # dynamics feasibility constraint

tau_limits = prb.createIntermediateConstraint("tau_limits", tau) 
tau_limits.setBounds(-tau_lim, tau_lim)  # setting input limits

prb.createConstraint("foot_tip_on_ground", foot_tip_position[2],
                     nodes=range(0, n_takeoff + 1))  # foot on the ground during the contact phase

no_grnd_tip_penetration = prb.createIntermediateConstraint("no_grnd_tip_penetration", foot_tip_position[2],nodes=range(n_takeoff+1, n_nodes + 1))  # dynamics feasibility constraint
no_grnd_tip_penetration.setLowerBounds(0)
no_grnd_tip_penetration.setUpperBounds(cs.inf)

prb.createConstraint("forward_hip_vel", v_hip[1]-forward_vel)  # keep a constant horizontal velocity of the hip center

prb.createConstraint("GRF_zero", f_contact,
                     nodes=range(n_takeoff, n_nodes))  # 0 GRF during flight

prb.createIntermediateConstraint("no_foot_slip_during_contact", v_foot_tip[1] , nodes=range(0,n_takeoff + 1)) # restore leg position and hip height at the end of the control horizon

prb.createIntermediateConstraint("periodic_position", q_p[1:4] - q_p_init[1:4], nodes=[0,n_nodes-1]) # restore leg position and hip height at the end of the control horizon
prb.createIntermediateConstraint("periodic_velocity", q_p_dot[1:4] - q_p_dot_init[1:4], nodes=[0,n_nodes-1]) # restore leg position and hip height at the end of the control horizon

i_q_hip=prb.createIntermediateConstraint("quadrature_current_hip", (hip_rotor_axial_MoI*q_p_ddot[2]/hip_red_ratio+tau[2]*hip_red_ratio/hip_efficiency)*1.0/hip_K_t)  # i_q hip less than the maximum allowed value
i_q_hip.setBounds(-hip_I_peak, hip_I_peak)  # setting input limits

i_q_knee=prb.createIntermediateConstraint("quadrature_current_knee", (knee_rotor_axial_MoI*q_p_ddot[3]/knee_red_ratio+tau[3]*knee_red_ratio/knee_efficiency)*1.0/knee_K_t)  # i_q knee less than the maximum allowed value
i_q_knee.setBounds(-knee_I_peak, knee_I_peak)  # setting input limits

print(n_takeoff+round((n_nodes-n_takeoff)/2))

prb.createIntermediateConstraint("tip_ground_clearance", foot_tip_position[2]-tip_ground_clearance, nodes=n_takeoff+round((n_nodes-n_takeoff)/2)) # restore leg position and hip height at the end of the control horizon

prb.createIntermediateConstraint("vertical_takeoff_vel", v_foot_tip[1], nodes=n_takeoff+1) # restore leg position and hip height at the end of the control horizon

prb.createIntermediateConstraint("vertical_touchdown_vel", v_foot_tip[1], nodes=n_nodes) # restore leg position and hip height at the end of the control horizon

##############################################

## Costs
prb.createIntermediateCost("min_f_contact", weight_contact_cost * cs.sumsqr(f_contact))

prb.createIntermediateCost("min_q_ddot", weight_q_ddot * cs.sumsqr(q_p_ddot[2:4]))  # minimizing the joint accelerations ("responsiveness" of the trajectory)

prb.createIntermediateCost("min_fictitious_actuation", weight_fictitious_actuation * cs.sumsqr(q_p_ddot[0:2]))  # minimizing the fictitious inputs (prismatic guides)

# prb.createIntermediateCost("overall_forward_vel", weight_forward_vel *cs.sumsqr(q_p_dot[0]-forward_vel))  # minimizing the joint accelerations ("responsiveness" of the trajectory)

# prb.createIntermediateCost("keep_the_hip_current_down_dammit", weight_hip_i_d * cs.sumsqr((hip_rotor_axial_MoI*q_p_ddot[1]/hip_red_ratio+tau[1]*hip_red_ratio/hip_efficiency)*1.0/hip_K_t))
# prb.createIntermediateCost("keep_the_knee_current_down_dammit", weight_knee_i_d * cs.sumsqr((knee_rotor_axial_MoI*q_p_ddot[2]/knee_red_ratio+tau[2]*knee_red_ratio/knee_efficiency)*1.0/knee_K_t))

##############################################

## Creating the solver and solving the problem
slvr = solver.Solver.make_solver(slvr_name, prb, slvr_opt) 
t = time.time()
slvr.solve()  # solving
solution_time = time.time() - t
print(f'solved in {solution_time} s')
solution = slvr.getSolutionDict() # extracting solution

##############################################

# Post-processing and solution storage
joint_names = urdf_awesome_leg.joint_names()
joint_names.remove("universe")  # removing the "universe joint"

cnstr_opt = slvr.getConstraintSolutionDict()

i_q=(hip_rotor_axial_MoI*solution["q_p_ddot"][2:4,:]/hip_red_ratio+cnstr_opt["tau_limits"][2:4,:]*hip_red_ratio/hip_efficiency)*1.0/hip_K_t

foot_tip_position_init_num=fk_foot(q=solution["q_p"][:,0])["ee_pos"]
hip_position_init_num=fk_hip(q=solution["q_p"][:,0])["ee_pos"]

solution_GRF = solution["f_contact"]
solution_q_p = solution["q_p"]
solution_foot_tip_position = fk_foot(q=solution_q_p)["ee_pos"][2,:].toarray()  # foot position
solution_hip_position = fk_hip(q=solution_q_p)["ee_pos"][2,:].toarray()   # hip position
init_solution_foot_tip_position_aux = np.tile(foot_tip_position_init_num,(1,n_nodes+1))
init_solution_hip_position_aux = np.tile(hip_position_init_num,(1,n_nodes+1))
solution_v_foot_tip = dfk_foot(q=solution["q_p"], qdot=solution["q_p_dot"])["ee_vel_linear"]  # foot velocity
solution_v_foot_hip = dfk_hip(q=solution["q_p"], qdot=solution["q_p_dot"])["ee_vel_linear"]  # foot velocity

useful_solutions={"q_p":solution["q_p"][2:4,:],"q_p_dot":solution["q_p_dot"][2:4,:], "q_p_ddot":solution["q_p_ddot"][2:4,:],
                 "tau":cnstr_opt["tau_limits"][2:4,:], "f_contact":solution["f_contact"], "i_q":i_q, "dt_opt":slvr.getDt(),
                 "foot_tip_height":np.transpose(solution_foot_tip_position-init_solution_foot_tip_position_aux[2,:]), 
                 "hip_height":np.transpose(solution_hip_position-init_solution_hip_position_aux[2,:]), 
                 "tip_velocity":np.transpose(np.transpose(solution_v_foot_tip)),
                 "hip_velocity":np.transpose(np.transpose(solution_v_foot_hip)),
                 "sol_time":solution_time}

##
ms.store(useful_solutions) # saving solution data to file
    
shutil.copyfile(rospackage.get_path("awesome_leg_pholus")+opt_res_rel_path+"/horizon_offline_solver.mat", target+"/horizon_offline_solver.mat")

if save_sol_as_init: # save the solution as the initialization for the next sim
    ms_opt_init.store(useful_solutions) # saving initialization data to file    
    shutil.copyfile(rospackage.get_path("awesome_leg_pholus")+opt_res_rel_path+"/horizon_offline_solver_init.mat", target+"horizon_offline_solver_init.mat")

################### RESAMPLING (necessary because dt is variable) #####################à
q_sym = cs.SX.sym('q', n_q)
q_dot_sym = cs.SX.sym('q_dot', n_v)
q_ddot_sym = cs.SX.sym('q_ddot', n_v)
x, x_dot = utils.double_integrator(q_sym, q_dot_sym, q_ddot_sym)

dae = {'x': x, 'p': q_ddot_sym, 'ode': x_dot, 'quad': 1}

dt_res = 0.01
sol_contact_map = dict(tip=solution["f_contact"])  # creating a contact map for applying the input to the foot

# print(solution)

p_res, v_res, a_res, frame_res_force_mapping, tau_res = resampler_trajectory.resample_torques(solution["q_p"],
                                                                                              solution["q_p_dot"],
                                                                                              solution["q_p_ddot"],
                                                                                              slvr.getDt().flatten(),
                                                                                              dt_res,
                                                                                              dae, sol_contact_map,
                                                                                              urdf_awesome_leg,
                                                                                              casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)

rpl_traj = replay_trajectory(dt_res, joint_names, p_res)  # replaying the (resampled) trajectory

# v_hip_forward = dfk_hip(q=p_res,qdot=v_res)["ee_vel_linear"][1,:]  # hip position
# print(v_hip_forward)

rpl_traj.replay(is_floating_base=False)

time_vector_res = np.zeros([p_res.shape[1]])
for i in range(1, p_res.shape[1] - 1):
    time_vector_res[i] = time_vector_res[i - 1] + dt_res

p_tip_res = fk_foot(q=p_res)["ee_pos"]  # foot vertical position
p_hip_res = fk_hip(q=p_res)["ee_pos"]  # hip position

