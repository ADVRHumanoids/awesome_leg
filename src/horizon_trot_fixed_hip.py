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

tanh_coeff = rospy.get_param("horizon/horizon_i_q_estimator/tanh_coeff")  # coefficient used by the approximated sign function ( sign = atanh(tanh_coeff * x) )
    
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
shutil.copyfile(rospackage.get_path("awesome_leg_pholus")+"/src"+"/"+scibidibi.path.basename(__file__), media_target+"/"+scibidibi.path.basename(__file__)) # saving a copy of this script for future reference and debugging

if save_sol_as_init: # save the solution as the initialization for the next sim
    ms_opt_init = mat_storer.matStorer(opt_res_path + "/horizon_offline_solver_init.mat")

if employ_opt_init: # initialize variables with the previously saved solution
    ms_load_path = opt_res_path+"/horizon_offline_solver_init.mat"
    ms_load = mat_storer.matStorer(ms_load_path)
    shutil.copyfile(ms_load_path, media_target) # copying used init to folder for reference and debugging
    loaded_sol = ms_load.load() # loading the solution dictionary

##################### LOADING THE URDF #########################

urdf = open(urdf_path, "r").read() # read the URDF
urdf_awesome_leg = casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn(urdf) # initialize casady_kin_dyn (based on Casadi + Pinocchio) object 

n_q = urdf_awesome_leg.nq()  # number of joints
n_v = urdf_awesome_leg.nv()  # number of dofs

lbs = urdf_awesome_leg.q_min()
ubs = urdf_awesome_leg.q_max()

print("lbs: ", lbs)
print("ubs: ", ubs)

##################### LOADING SOLVER PARAMETERS FROM THE ROS PARAMETER SERVER #########################

## Some problem parameters:
n_nodes = rospy.get_param("horizon/horizon_solver/problem_settings/n_nodes")  # optimization horizon
n_takeoff =rospy.get_param("horizon/horizon_solver/problem_settings/n_takeoff")   # instant of takeoff
dt= rospy.get_param("horizon/horizon_solver/problem_settings/dt")   # using a fixed dt

forward_vel= rospy.get_param("horizon/horizon_solver/problem_settings/forward_vel") # desired forward hip horizontal velocity
tip_ground_clearance= rospy.get_param("horizon/horizon_solver/problem_settings/tip_ground_clearance") # desired foot tip vertical clearance during the flight phase
flight_phase_tip_clearance_percentage=rospy.get_param("horizon/horizon_solver/problem_settings/flight_phase_tip_clearance_percentage") # when to achieve the required clearance, w.r.t. the flight phase

## Cost weights:
weight_q_ddot = rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/small_q_p_ddot")# minimizing joint accelerations
weight_forward_vel=rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/weight_forward_vel") # maximizing the jump height (measured at the tip)
weight_large_tip_vert_excursion=rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/weight_forward_vel") 
weight_min_input_diff=rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/weight_min_input_diff") 

## Solver options:
slvr_opt = {"ipopt.tol": rospy.get_param("horizon/horizon_solver/problem_settings/solver/tolerance"), 
"ipopt.max_iter": rospy.get_param("horizon/horizon_solver/problem_settings/solver/max_iter"), 
"ipopt.linear_solver": rospy.get_param("horizon/horizon_solver/problem_settings/solver/linear_solver_name")} 
slvr_name=rospy.get_param("horizon/horizon_solver/problem_settings/solver/name") # options: "blocksqp", "ipopt", "ilqr", "gnsqp", 

## Actuator properties (see actuators.yaml for a brief description of most of the parameters):
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
    
##################### SETTING THE OPT PROBLEM THROUGH HORIZON #########################

## Initializing som object for bounds, etc.. 
tau_lim = np.array([cs.inf, cs.inf])  # effort limits

## Initialization of a problem object
prb = Problem(n_nodes) 

## Transcriptor settings
transcriptor_name = "multiple_shooting"  # other option: "direct_collocation"
trans_opt = dict(integrator="RK4")  # dictionary with the chosen integrator name

## Setting the problem dt 
prb.setDt(dt) # using a constant and fixed dt

## Creating the state variables (and assigning an initialization, if provided)
q_p = prb.createStateVariable("q_p", n_q)
q_p_dot = prb.createStateVariable("q_p_dot",
                                  n_v) 
q_p_ddot = prb.createInputVariable("q_p_ddot", n_v)  # using joint accelerations as an input variable

q_p.setBounds(lbs, ubs) # hip
q_p_dot.setBounds(-omega_max_nl_af112, omega_max_nl_af112)

if employ_opt_init: # using initial guesses (if this option is set in the horizon config file)
    q_p.setInitialGuess(loaded_sol["q_p"])
    q_p_dot.setInitialGuess(loaded_sol["q_p_dot"])    
    q_p_ddot.setInitialGuess(loaded_sol["q_p_ddot"])

## Building the full state
x, xdot = utils.double_integrator(q_p, q_p_dot, q_p_ddot)  

## The initial conditions are free for the optimizer to choose
q_p_init = prb.createSingleVariable("q_p_init", n_q)  # single variable for leaving the initial condition free
q_p_dot_init = prb.createSingleVariable("q_p_dot_init", n_q)  # single variable for leaving the initial (velocity) condition free

## Additional input variable for the contact forces

prb.setDynamics(xdot)  # setting the dynamics we are interested of in the problem object (xdot)

## Creating the transcriptor object, using the previously set option, problem and transciptor name
trscptr = transcriptor.Transcriptor.make_method(transcriptor_name, prb, trans_opt)  # setting the transcriptor

## Obtaining some relevant objects

tau = kin_dyn.InverseDynamics(urdf_awesome_leg).call(q_p, q_p_dot,q_p_ddot) # joint efforts through the inverse dynamics

# foot tip pos
fk_foot = cs.Function.deserialize(urdf_awesome_leg.fk("tip"))
foot_tip_position_init = fk_foot(q=q_p_init)["ee_pos"]  # foot initial position
foot_tip_position = fk_foot(q=q_p)["ee_pos"]  # foot position

# foot tip vel
dfk_foot = cs.Function.deserialize(urdf_awesome_leg.frameVelocity("tip", casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
v_foot_tip = dfk_foot(q=q_p, qdot=q_p_dot)["ee_vel_linear"]  # foot velocity

## Constraints

# tau bounds
tau_limits = prb.createIntermediateConstraint("tau_limits", tau) 
tau_limits.setBounds(-tau_lim, tau_lim)  # setting input limits

# foot on the ground during the contact phase
# prb.createConstraint("foot_tip_on_ground", foot_tip_position[2], nodes=range(0, n_takeoff))  

# Stay above the ground during the flight phase
no_grnd_tip_penetration = prb.createIntermediateConstraint("no_grnd_tip_penetration", foot_tip_position[2],nodes=range(n_takeoff+1, n_nodes + 1))  
no_grnd_tip_penetration.setLowerBounds(0)
no_grnd_tip_penetration.setUpperBounds(cs.inf)

# The generated trajectory must be periodic
prb.createIntermediateConstraint("periodic_position", q_p - q_p_init, nodes=[0,n_nodes]) 
prb.createIntermediateConstraint("periodic_velocity", q_p_dot - q_p_dot_init, nodes=[0,n_nodes]) 

# Keep the ESTIMATED quadrature currents within bounds
compensated_tau = []
i_q_cnstr = []
for i in range(n_q):
    compensated_tau.append(tau[i] + K_d0[i] * np.tanh( tanh_coeff * q_p_dot[i]) + K_d1[i] * q_p_dot[i])
    i_q = (rotor_axial_MoI[i] * q_p_ddot[i] / red_ratio[i] + compensated_tau[i] * red_ratio[i] / efficiency[i]) / K_t[i]
    i_q_cnstr.append( prb.createIntermediateConstraint("quadrature_current" + actuator_names[i], i_q) )
    i_q_cnstr[i].setBounds(-I_peak[i], I_peak[i])  # setting input limits

# Imposing a vertical tip clearance between the foot tip and the ground
prb.createIntermediateConstraint("tip_ground_clearance", foot_tip_position[2] - tip_ground_clearance, nodes = n_takeoff + round(1*(n_nodes - n_takeoff) * flight_phase_tip_clearance_percentage)) 

# Foot tip velocity only vertical @takeoff and @touchdown
prb.createIntermediateConstraint("vertical_takeoff_vel", v_foot_tip[1], nodes = n_takeoff + 1)
prb.createIntermediateConstraint("vertical_touchdown_vel", v_foot_tip[1], nodes = n_nodes) 

## Costs:

# Minimize the joint accelerations ("responsiveness" of the trajectory)
prb.createIntermediateCost("min_q_ddot", weight_q_ddot * cs.sumsqr(q_p_ddot))  
# Keep the forward hip velocity more or less constant
prb.createIntermediateCost("overall_forward_vel", weight_forward_vel *cs.sumsqr(v_foot_tip[1]-forward_vel), nodes = range(n_takeoff+1, n_nodes + 1))  
# Penalize the difference between successive control inputs
prb.createIntermediateCost("min_input_diff", weight_min_input_diff * cs.sumsqr(q_p_ddot-q_p_ddot.getVarOffset(-1)),nodes = range(1, n_nodes))  

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
i_q = np.zeros((n_q, len(solution["q_p_ddot"][0, :])))
for i in range(n_q):
    i_q[i, :] = (rotor_axial_MoI[i] * solution["q_p_ddot"][i, :] / red_ratio[i] + cnstr_opt["tau_limits"][i, :] * red_ratio[i] / efficiency[i])*1.0/K_t[i]

# Hip and knee initial positions, according to the in. cond. chosen by the optimizer
foot_tip_position_init_num=  fk_foot(q = solution["q_p"][:,0])["ee_pos"]

# q_p obtained from the solution
solution_q_p = solution["q_p"]

# Foot position on the computed trajectory
solution_foot_tip_position = fk_foot(q=solution_q_p)["ee_pos"][2,:].toarray()  

# Auxiliary array for computing the difference of the foot tip/hip position w.r.t. its initial pos.
init_solution_foot_tip_position_aux = np.tile(foot_tip_position_init_num,(1, n_nodes + 1))

# Foot tip velocity on the trajectory
solution_v_foot_tip = dfk_foot(q = solution["q_p"], qdot = solution["q_p_dot"])["ee_vel_linear"]  

# Building up a custom solution dictionary for post-processing and debug
useful_solutions={"q_p":solution["q_p"], "q_p_dot":solution["q_p_dot"], "q_p_ddot":solution["q_p_ddot"],
                 "tau":cnstr_opt["tau_limits"], "f_contact": np.zeros((0, 0)), "i_q": i_q, "dt_opt":slvr.getDt(),
                 "foot_tip_height":np.transpose(solution_foot_tip_position-init_solution_foot_tip_position_aux[2, :]), 
                 "hip_height":np.zeros((0, 0)), 
                 "tip_velocity":np.transpose(np.transpose(solution_v_foot_tip)),
                 "hip_velocity":np.zeros((0, 0)),
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

# Resampling
p_res, v_res, a_res, frame_res_force_mapping, tau_res = resampler_trajectory.resample_torques(solution["q_p"],
                                                                                              solution["q_p_dot"],
                                                                                              solution["q_p_ddot"],
                                                                                              slvr.getDt().flatten(),
                                                                                              dt_res,
                                                                                              dae, {},
                                                                                              urdf_awesome_leg)

# Replaying the (resampled) trajectory
rpl_traj = replay_trajectory(dt_res, joint_names, p_res)  
rpl_traj.replay(is_floating_base=False)

