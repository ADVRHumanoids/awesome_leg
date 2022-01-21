#!/usr/bin/env python3

##################### IMPORTS #########################

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

import os as scibidibi # "Why are you importing os with a silly name; are you crazy?", you may ask and I would tell you "Perchè si"
import shutil
from datetime import date

from horizon.ros.replay_trajectory import *

##################### LOADING SOLVER PARAMETERS FROM THE ROS PARAMETER SERVER #########################

urdf_path = rospy.get_param("/horizon/urdf_path")  # urdf relative path (wrt to the package)

media_path = rospy.get_param("/horizon/media_path")  # urdf relative path (wrt to the package)
simulation_name = rospy.get_param("/horizon/simulation_name")  # simulation name (used to save different simulations to different locations)

opt_res_path = rospy.get_param("/horizon/opt_results_path")  # urdf relative path (wrt to the package)

save_sol_as_init = rospy.get_param("horizon/horizon_solver/save_sol_as_init")  # if true, the solution is also saved as a candidate for future optimization initializations
employ_opt_init = rospy.get_param("horizon/horizon_solver/employ_opt_init")  # if true, the solution is also saved as a candidate for future optimization initializations

##################### INITIALIZING OBJECTS (ad other stuff) FOR STORAGE AND POST-PROCESSING PURPOSES #########################

## Getting/setting some useful variables
rospackage=rospkg.RosPack() # Absolute path to the ROS package
today = date.today() # current date; used for storing different simulations
today_is = today.strftime("%d-%m-%Y") # time to string 
config_path=rospackage.get_path("awesome_leg_pholus")+"/config/" # absolute path of the configuration files

## Creating folders for saving plots and other data (if not already existing). This folders are also used by horizon_plot.py (plots graphs)
media_target=media_path+"/"+today_is+"/"+simulation_name # target directory where a copy of the opt results is placed (and also some useful graphs)
opt_res_target=opt_res_path+"/horizon_offline_solver.mat" # auxiliary target directory for opt results, where the LAST solution is placed

if  (not scibidibi.path.isdir(media_target)): # if media target does not exist, create it
    scibidibi.makedirs(media_target)

ms = mat_storer.matStorer(opt_res_target)# initializing storer object for opt results

shutil.copyfile(config_path+"actuators.yaml", media_target+"/actuators.yaml") # saving a copy of the used actuator config file for future reference and debugging
shutil.copyfile(config_path+"horizon_trot_sliding_all.yaml", media_target+"/horizon.yaml") # saving a copy of the used horizon config file for future reference and debugging
shutil.copyfile(config_path+"xbot2_sim_config.yaml", media_target+"/xbot2.yaml") # saving a copy of the used xbot config file for future reference and debugging
shutil.copyfile(rospackage.get_path("awesome_leg_pholus")+"/src"+"/"+scibidibi.path.basename(__file__), media_target+"/"+scibidibi.path.basename(__file__)) # saving a copy of this script for future reference and debugging

if save_sol_as_init: # save the solution as the initialization for the next sim
    ms_opt_init = mat_storer.matStorer(opt_res_path+"/horizon_offline_solver_init.mat")

if employ_opt_init: # initialize variables with the previously saved solution
    ms_load_path=opt_res_path+"/horizon_offline_solver_init.mat"
    ms_load = mat_storer.matStorer(ms_load_path)
    shutil.copyfile(ms_load_path, media_target) # copying used init to folder for reference and debugging
    loaded_sol=ms_load.load() # loading the solution dictionary

##################### LOADING SOLVER PARAMETERS FROM THE ROS PARAMETER SERVER #########################

## Some problem parameters:
n_nodes = rospy.get_param("horizon/horizon_solver/problem_settings/n_nodes")  # optimization horizon
n_takeoff =rospy.get_param("horizon/horizon_solver/problem_settings/n_takeoff")   # instant of takeoff
dt= rospy.get_param("horizon/horizon_solver/problem_settings/dt")   # using a fixed dt

forward_vel= rospy.get_param("horizon/horizon_solver/problem_settings/forward_vel") # desired forward hip horizontal velocity
tip_ground_clearance= rospy.get_param("horizon/horizon_solver/problem_settings/tip_ground_clearance") # desired foot tip vertical clearance during the flight phase
flight_phase_tip_clearance_percentage=rospy.get_param("horizon/horizon_solver/problem_settings/flight_phase_tip_clearance_percentage") # when to achieve the required clearance, w.r.t. the flight phase

## Bounds
test_rig_lb= rospy.get_param("horizon/horizon_solver/problem_settings/test_rig/lb") # lower bound of the test rig excursion
test_rig_ub=rospy.get_param("horizon/horizon_solver/problem_settings/test_rig/ub") # upper bound of the test rig excursion
hip_jnt_lb= rospy.get_param("horizon/horizon_solver/problem_settings/hip_joint/lb") 
hip_jnt_ub=rospy.get_param("horizon/horizon_solver/problem_settings/hip_joint/ub")
knee_jnt_lb= rospy.get_param("horizon/horizon_solver/problem_settings/knee_joint/lb") 
knee_jnt_ub=rospy.get_param("horizon/horizon_solver/problem_settings/knee_joint/ub")

## Cost weights:
weight_contact_cost = rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/contact_force")  # minimizing the contact force
weight_fictitious_actuation= rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/weight_fictitious_actuation") # minimizing the fictitious actuations
weight_q_ddot = rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/small_q_p_ddot")# minimizing joint accelerations
weight_forward_vel=rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/weight_forward_vel") # maximizing the jump height (measured at the tip)
weight_large_tip_vert_excursion=rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/weight_forward_vel") 
weight_min_input_diff=rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/weight_min_input_diff") 

# weight_postural_cost = rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/terminal_postural") # cost to restore the initial position at the end of the control horizon
# weight_hip_i_d=rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/weight_hip_i_d") # minimizing hip i_d
# weight_knee_i_d=rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/weight_knee_i_d") # minimizing hip i_d
# solver options

## Solver options:
slvr_opt = {"ipopt.tol": rospy.get_param("horizon/horizon_solver/problem_settings/solver/tolerance"), 
"ipopt.max_iter": rospy.get_param("horizon/horizon_solver/problem_settings/solver/max_iter"), 
"ipopt.linear_solver": rospy.get_param("horizon/horizon_solver/problem_settings/solver/linear_solver_name")} 
slvr_name=rospy.get_param("horizon/horizon_solver/problem_settings/solver/name") # options: "blocksqp", "ipopt", "ilqr", "gnsqp", 

## Actuator properties (see actuators.yaml for a brief description of most of the parameters):

# Hip actuator:
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

##################### LOADING THE URDF #########################

urdf = open(urdf_path, "r").read() # read the URDF
urdf_awesome_leg = casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn(urdf) # initialize casady_kin_dyn (based on Casadi + Pinocchio) object 

n_q = urdf_awesome_leg.nq()  # number of joints
n_v = urdf_awesome_leg.nv()  # number of dofs

##################### SETTING THE OPT PROBLEM THROUGH HORIZON #########################

## Initializing som object for bounds, etc.. 
leg_joint_vel_lim=np.array([hip_omega_max_nl_af112,hip_omega_max_nl_af112]) # jont velocity limits (of the leg)
tau_lim = np.array([cs.inf, cs.inf, cs.inf, cs.inf])  # effort limits (excluding the fictious linear actuations on the hip joint)
tau_lim_stance_phase= np.array([0, 0, tau_lim[2], tau_lim[3]]) # during the ground contact phase do not apply inputs to the linear rails

## Initialization of a problem object
prb = Problem(n_nodes) 

## Transcriptor settings
transcriptor_name = "multiple_shooting"  # other option: "direct_collocation"
trans_opt = dict(integrator="RK4")  # dictionary with the chosen integrator name

## Setting the problem dt 
prb.setDt(dt) # using a constant and fixed dt

# Setting a piecewise variable dt (not used)
# dt1 = prb.createSingleVariable("dt1", 1)  # dt for the foot tip topuch phase
# dt2 = prb.createSingleVariable("dt2", 1)  # dt for the flight phase
# dt1.setBounds(dt_lb, dt_ub)  # bounds on dt1
# dt2.setBounds(dt_lb, dt_ub)  # bounds on dt2
# Dt=[dt1]*n_takeoff+[dt2]*(n_nodes-n_takeoff) # holds the complete time list
# prb.setDt(Dt)

## Creating the state variables (and assigning an initialization, if provided)
q_p = prb.createStateVariable("q_p", n_q)
q_p_dot = prb.createStateVariable("q_p_dot",
                                  n_v) 
q_p_ddot = prb.createInputVariable("q_p_ddot", n_v)  # using joint accelerations as an input variable

q_p_dot[2:4].setBounds(-leg_joint_vel_lim, leg_joint_vel_lim)
q_p[1].setBounds(test_rig_lb, test_rig_ub) # test rig excursion
q_p[2].setBounds(hip_jnt_lb, hip_jnt_ub) # hip
q_p[3].setBounds(knee_jnt_lb, knee_jnt_ub) # knee


if employ_opt_init: # using initial guesses (if this option is set in the horizon config file)
    q_p[1:].setInitialGuess(loaded_sol["q_p"])
    q_p_dot[1:].setInitialGuess(loaded_sol["q_p_dot"])    
    q_p_ddot[1:].setInitialGuess(loaded_sol["q_p_ddot"])

## Building the full state
x, xdot = utils.double_integrator(q_p, q_p_dot, q_p_ddot)  

## Additional input variable for the contact forces
f_contact = prb.createInputVariable("f_contact", 3)  # dimension 3
if employ_opt_init:
    f_contact.setInitialGuess(loaded_sol["f_contact"])
else:
    f_contact[2].setInitialGuess(100.0)

f_contact[2].setLowerBounds(0)  # the vertical component of f_contact needs to be always positive

## Creating a contact dictionary for applying the input to the foot
contact_map = dict(tip=f_contact)  

## The initial conditions are free for the optimizer to choose
q_p_init = prb.createSingleVariable("q_p_init", n_q)  # single variable for leaving the initial condition free
q_p_dot_init = prb.createSingleVariable("q_p_dot_init", n_q)  # single variable for leaving the initial (velocity) condition free

## Additional input variable for the contact forces

prb.setDynamics(xdot)  # setting the dynamics we are interested of in the problem object (xdot)

## Creating the transcriptor object, using the previously set option, problem and transciptor name
trscptr = transcriptor.Transcriptor.make_method(transcriptor_name, prb, trans_opt)  # setting the transcriptor

## Obtaining some relevant objects

tau = kin_dyn.InverseDynamics(urdf_awesome_leg, contact_map.keys(),casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED).call(q_p, q_p_dot,q_p_ddot,contact_map) # joint efforts through the inverse dynamics

# hip pos
fk_hip = cs.Function.deserialize(urdf_awesome_leg.fk("hip1_1"))  # deserializing (basically, reading from memory)
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

## Constraints

# tau bounds
tau_limits = prb.createIntermediateConstraint("tau_limits", tau) 
tau_limits.setBounds(-tau_lim, tau_lim)  # setting input limits
tau_limits.setBounds(-tau_lim_stance_phase, tau_lim_stance_phase, nodes=range(0,n_takeoff-1))  # overriding the main input limits during the stance phase (no linear rail actuation)

# foot on the ground during the contact phase
prb.createConstraint("foot_tip_on_ground", foot_tip_position[2], nodes=range(0, n_takeoff + 1))  

# Stay above the ground level during the flight phase
no_grnd_tip_penetration = prb.createIntermediateConstraint("no_grnd_tip_penetration", foot_tip_position[2],nodes=range(n_takeoff+1, n_nodes + 1))  
no_grnd_tip_penetration.setLowerBounds(0)
no_grnd_tip_penetration.setUpperBounds(cs.inf)

# 0 GRF during flight
prb.createConstraint("GRF_zero", f_contact,
                     nodes=range(n_takeoff, n_nodes))  

# No foot slip during ground contact 
prb.createIntermediateConstraint("no_foot_slip_during_contact", v_foot_tip[1] , nodes=range(0,n_takeoff + 1))

# The generated trajectory must be periodic
prb.createIntermediateConstraint("periodic_position", q_p[1:4] - q_p_init[1:4], nodes=[0,n_nodes]) 
prb.createIntermediateConstraint("periodic_velocity", q_p_dot[1:4] - q_p_dot_init[1:4], nodes=[0,n_nodes]) 

# Keep the ESTIMATED quadrature currents within bounds

# i_q hip less than the maximum allowed value
i_q_hip=prb.createIntermediateConstraint("quadrature_current_hip", (hip_rotor_axial_MoI*q_p_ddot[2]/hip_red_ratio+tau[2]*hip_red_ratio/hip_efficiency)*1.0/hip_K_t)  
i_q_hip.setBounds(-hip_I_peak, hip_I_peak)  # setting input limits

# i_q knee less than the maximum allowed value
i_q_knee=prb.createIntermediateConstraint("quadrature_current_knee", (knee_rotor_axial_MoI*q_p_ddot[3]/knee_red_ratio+tau[3]*knee_red_ratio/knee_efficiency)*1.0/knee_K_t)  
i_q_knee.setBounds(-knee_I_peak, knee_I_peak)  # setting input limits

# Imposing a vertical tip clearance between the foot tip and the ground
prb.createIntermediateConstraint("tip_ground_clearance", foot_tip_position[2]-tip_ground_clearance, nodes=n_takeoff+round(1*(n_nodes-n_takeoff)*flight_phase_tip_clearance_percentage)) 

# Foot tip velocity only vertical @takeoff and @touchdown
prb.createIntermediateConstraint("vertical_takeoff_vel", v_foot_tip[1], nodes=n_takeoff+1)
prb.createIntermediateConstraint("vertical_touchdown_vel", v_foot_tip[1], nodes=n_nodes) 

# prb.createConstraint("forward_hip_vel", v_hip[1]-forward_vel)  # keep a constant horizontal velocity of the hip center (hard constraint, can make the problem unfeasible)

## Costs:

# minimizing the fictitious inputs (on the prismatic guides during flight phase)
prb.createIntermediateCost("min_fictitious_actuation", weight_fictitious_actuation * cs.sumsqr(q_p_ddot[0:2]))
# Minimize the contact force
prb.createIntermediateCost("min_f_contact", weight_contact_cost * cs.sumsqr(f_contact))
# Minimize the joint accelerations ("responsiveness" of the trajectory)
prb.createIntermediateCost("min_q_ddot", weight_q_ddot * cs.sumsqr(q_p_ddot[2:4]))  
# Keep the forward hip velocity more or less constant
prb.createIntermediateCost("overall_forward_vel", weight_forward_vel *cs.sumsqr(q_p_dot[0]-forward_vel))  
# Penalize the difference between successive control inputs
prb.createIntermediateCost("min_input_diff", weight_min_input_diff * cs.sumsqr(q_p_ddot-q_p_ddot.getVarOffset(-1)),nodes=range(1,n_nodes))  

# prb.createIntermediateCost("keep_the_hip_current_down_dammit", weight_hip_i_d * cs.sumsqr((hip_rotor_axial_MoI*q_p_ddot[1]/hip_red_ratio+tau[1]*hip_red_ratio/hip_efficiency)*1.0/hip_K_t))
# prb.createIntermediateCost("keep_the_knee_current_down_dammit", weight_knee_i_d * cs.sumsqr((knee_rotor_axial_MoI*q_p_ddot[2]/knee_red_ratio+tau[2]*knee_red_ratio/knee_efficiency)*1.0/knee_K_t))

## Creating the solver and solving the problem:

slvr = solver.Solver.make_solver(slvr_name, prb, slvr_opt) 
t = time.time()
slvr.solve()  # solving
solution_time = time.time() - t
print(f'solved in {solution_time} s') # rough estimate of the solution time
solution = slvr.getSolutionDict() # extracting the solution

## Post-processing and solution storage:

# removing the "universe joint"
joint_names = urdf_awesome_leg.joint_names()
joint_names.remove("universe")  

# Getting the torque inputs (which were enforced though a dynamics feasibility contraint)
cnstr_opt = slvr.getConstraintSolutionDict() 

# Hip and knee quadrature current estimation
i_q=(hip_rotor_axial_MoI*solution["q_p_ddot"][2:4,:]/hip_red_ratio+cnstr_opt["tau_limits"][2:4,:]*hip_red_ratio/hip_efficiency)*1.0/hip_K_t

# Hip and knee initial positions, according to the in. cond. chosen by the optimizer
foot_tip_position_init_num=fk_foot(q=solution["q_p"][:,0])["ee_pos"]
hip_position_init_num=fk_hip(q=solution["q_p"][:,0])["ee_pos"]

# GRF obtained from the solution
solution_GRF = solution["f_contact"]

# q_p obtained from the solution
solution_q_p = solution["q_p"]

# Foot position on the computed trajectory
solution_foot_tip_position = fk_foot(q=solution_q_p)["ee_pos"][2,:].toarray()  

# Hip position on the computed trajectory
solution_hip_position = fk_hip(q=solution_q_p)["ee_pos"][2,:].toarray()   # hip position

# Auxiliary array for computing the difference of the foot tip/hip position w.r.t. its initial pos.
init_solution_foot_tip_position_aux = np.tile(foot_tip_position_init_num,(1,n_nodes+1))
init_solution_hip_position_aux = np.tile(hip_position_init_num,(1,n_nodes+1))

# Foot ti/hip velocity on the trajectory
solution_v_foot_tip = dfk_foot(q=solution["q_p"], qdot=solution["q_p_dot"])["ee_vel_linear"]  
solution_v_foot_hip = dfk_hip(q=solution["q_p"], qdot=solution["q_p_dot"])["ee_vel_linear"]  # foot velocity

# Building up a custom solution dictionary for post-processing and debug
useful_solutions={"q_p":solution["q_p"][2:4,:],"q_p_dot":solution["q_p_dot"][2:4,:], "q_p_ddot":solution["q_p_ddot"][2:4,:],
                 "tau":cnstr_opt["tau_limits"][2:4,:], "f_contact":solution["f_contact"], "i_q":i_q, "dt_opt":slvr.getDt(),
                 "foot_tip_height":np.transpose(solution_foot_tip_position-init_solution_foot_tip_position_aux[2,:]), 
                 "hip_height":np.transpose(solution_hip_position-init_solution_hip_position_aux[2,:]), 
                 "tip_velocity":np.transpose(np.transpose(solution_v_foot_tip)),
                 "hip_velocity":np.transpose(np.transpose(solution_v_foot_hip)),
                 "sol_time":solution_time}

# Saving solution data to opt_res_target folder
ms.store(useful_solutions) 

# Putting a copy of the opt results in the auxiliary folder (used, for ex., by the publisher node to read the trajectory)
shutil.copyfile(opt_res_target, media_target+"/horizon_offline_solver.mat")

# If set, save the solution as the initialization for the next sim
if save_sol_as_init: 
    ms_opt_init.store(useful_solutions) # saving initialization data to file    
    shutil.copyfile(opt_res_path+"/horizon_offline_solver_init.mat", media_target+"horizon_offline_solver_init.mat")


##  Resampling (necessary if dt is variable) 

q_sym = cs.SX.sym('q', n_q)
q_dot_sym = cs.SX.sym('q_dot', n_v)
q_ddot_sym = cs.SX.sym('q_ddot', n_v)
x, x_dot = utils.double_integrator(q_sym, q_dot_sym, q_ddot_sym)

dae = {'x': x, 'p': q_ddot_sym, 'ode': x_dot, 'quad': 1}

dt_res = 0.001 # resampling dt (has to be smaller than the smallest of the original dts)

sol_contact_map = dict(tip=solution["f_contact"])  # creating a contact map for applying the input to the foot

# Resampling
p_res, v_res, a_res, frame_res_force_mapping, tau_res = resampler_trajectory.resample_torques(solution["q_p"],
                                                                                              solution["q_p_dot"],
                                                                                              solution["q_p_ddot"],
                                                                                              slvr.getDt().flatten(),
                                                                                              dt_res,
                                                                                              dae, sol_contact_map,
                                                                                              urdf_awesome_leg,
                                                                                              casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)

# Replaying the (resampled) trajectory
rpl_traj = replay_trajectory(dt_res, joint_names, p_res)  
rpl_traj.replay(is_floating_base=False)

# Resampled time vector
time_vector_res = np.zeros([p_res.shape[1]])
for i in range(1, p_res.shape[1] - 1):
    time_vector_res[i] = time_vector_res[i - 1] + dt_res

# p_tip_res = fk_foot(q=p_res)["ee_pos"]  
# p_hip_res = fk_hip(q=p_res)["ee_pos"]  # hip position