#!/usr/bin/env python3

##################### Imports #########################

import os as scibidibi
import shutil
import time
from datetime import date

import casadi as cs
import casadi_kin_dyn
import numpy as np
import rospkg
import rospy
from horizon.problem import Problem
from horizon.ros.replay_trajectory import *
from horizon.solvers import solver
from horizon.transcriptions import transcriptor
from horizon.utils import kin_dyn, mat_storer, resampler_trajectory, utils

from datetime import datetime

##################### Reading the solution modes from config YAML config file #########################

save_sol_as_init = rospy.get_param("horizon/horizon_solver/save_sol_as_init")  # if true, the solution is also saved as a candidate for future optimization initializations
employ_opt_init = rospy.get_param("horizon/horizon_solver/employ_opt_init")  # if true, the solution is also saved as a candidate for future optimization initializations

opt_init_name = rospy.get_param("horizon/horizon_solver/opt_init_name") # name of the initialization file to be loaded

urdf_path = rospy.get_param("/horizon/urdf_path")  # urdf relative path (wrt to the package)

media_path = rospy.get_param("/horizon/media_path")  # urdf relative path (wrt to the package)
opt_res_path = rospy.get_param("/horizon/opt_results_path")  # urdf relative path (wrt to the package)

tanh_coeff = rospy.get_param("horizon/horizon_i_q_estimator/tanh_coeff")  # coefficient used by the approximated sign function ( sign = atanh(tanh_coeff * x) )

##################### Initializing objects for .mat storage #########################

## Getting/setting some useful variables
today = date.today()
today_is = today.strftime("%d-%m-%Y")
now = datetime.now()
current_time = now.strftime("_%H_%M_%S")

rospackage=rospkg.RosPack() # Only for taking the path to the leg package
config_path=rospackage.get_path("awesome_leg")+"/config/" # configuration files path
horizon_config_path = config_path + "horizon/"
## Creating folders for saving plots and other data (if not already existing). This folders are also used by horizon_plot.py

if  (not scibidibi.path.isdir(media_path+"/"+today_is)):
    scibidibi.makedirs(media_path+"/"+today_is)

if  (not scibidibi.path.isdir(media_path+"/"+today_is+"/jump_test/")):
    scibidibi.mkdir(media_path+"/"+today_is+"/jump_test")

## Various operations based on the selected options
sol_mat_name = rospy.get_param("/horizon/horizon_solver/sol_mat_name")
res_sol_mat_name = rospy.get_param("/horizon/horizon_solver/res_sol_mat_name")

ms = mat_storer.matStorer(opt_res_path + "/jump_test/horizon_offline_solver.mat")  # original opt. sol
ms_res = mat_storer.matStorer(opt_res_path + "/jump_test/horizon_offline_solver_res.mat") # resampled sol
ms_aux = mat_storer.matStorer(media_path + "/" + today_is + "/" + "/jump_test/" + sol_mat_name + current_time + ".mat") # additional location

target = media_path + "/" + today_is + "/jump_test/"

shutil.copyfile(config_path + "actuators.yaml", target + "actuators" + current_time + ".yaml") # saving config files for reference and future debugging
shutil.copyfile(horizon_config_path + "horizon_jump.yaml", target + "horizon" + current_time + ".yaml")
shutil.copyfile(urdf_path, target + "awesome_leg_complete_" + current_time + ".urdf")

if save_sol_as_init: # save the solution as the initialization for the next sim
    ms_opt_init = mat_storer.matStorer(opt_res_path + "/jump_test/" + opt_init_name + ".mat")
if employ_opt_init: # initialize variables with the previously saved solution
    ms_load_path = opt_res_path + "/jump_test/" + opt_init_name + ".mat"
    ms_load = mat_storer.matStorer(ms_load_path)
    shutil.copyfile(ms_load_path, target + "horizon_offline_solver_init" + current_time + ".mat") 
    loaded_sol=ms_load.load() # loading the solution dictionary

##################### LOADING SOLVER PARAMETERS FROM SERVER #########################

## Problem parameters, based on the chosen options

n_nodes = rospy.get_param("horizon/horizon_solver/problem_settings/n_nodes")  # optimization horizon
n_takeoff =rospy.get_param("horizon/horizon_solver/problem_settings/n_takeoff")   # instant of takeoff
n_touchdown =rospy.get_param("horizon/horizon_solver/problem_settings/n_touchdown")   # instant of touchdown

dt_lb=rospy.get_param("horizon/horizon_solver/problem_settings/dt_lb")   # dt lower bound 
dt_ub=rospy.get_param("horizon/horizon_solver/problem_settings/dt_ub")   # dt upper bound
test_rig_lb= rospy.get_param("horizon/horizon_solver/problem_settings/test_rig/lb") # lower bound of the test rig excursion
test_rig_ub=rospy.get_param("horizon/horizon_solver/problem_settings/test_rig/ub") # upper bound of the test rig excursion
q_p_init = rospy.get_param("horizon/horizon_solver/problem_settings/initial_conditions/q_p") # initial joint config (ideally it would be given from measurements)
q_p_dot_init = rospy.get_param("horizon/horizon_solver/problem_settings/initial_conditions/q_p_dot") # initial joint config (ideally it would be given from measurements)

jnt_limit_margin = abs(rospy.get_param("horizon/horizon_solver/problem_settings/jnt_limit_margin")) # margin to be added to joint limits 

# cost weights

weight_contact_cost = rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/contact_force")  # minimizing the contact force
weight_q_ddot = rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/small_q_p_ddot")# minimizing joint accelerations
weight_hip_height_jump = rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/big_hip_jump") # maximizing the jump height (measured at the hip)
weight_hip_height_target = rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/hip_jump_target") # minimizing the difference to a reference excursion
weight_tip_clearance=rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/big_foot_tip_clearance") # maximizing the jump height (measured at the tip)
weight_min_input_diff=rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/weight_min_input_diff") 

# solver options

slvr_opt = {"ipopt.tol": rospy.get_param("horizon/horizon_solver/problem_settings/solver/tolerance"),
    "ipopt.max_iter": rospy.get_param("horizon/horizon_solver/problem_settings/solver/max_iter")} 
slvr_name=rospy.get_param("horizon/horizon_solver/problem_settings/solver/name") # options: "blocksqp", "ipopt", "ilqr", "gnsqp", 

trans_name = rospy.get_param("horizon/horizon_solver/problem_settings/transcription/name")
trans_integrator = rospy.get_param("horizon/horizon_solver/problem_settings/transcription/integrator")

################# Getting the properties of the actuator from the ROS parameter server #########################

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

##################### SETTING THE OPT PROBLEM #########################

urdf = open(urdf_path, "r").read()
urdf_awesome_leg = casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn(urdf)

n_q = urdf_awesome_leg.nq()  # number of joints
n_v = urdf_awesome_leg.nv()  # number of dofs

jnt_lim_margin_array = np.tile(jnt_limit_margin, (n_q))
lbs = urdf_awesome_leg.q_min() + jnt_lim_margin_array
ubs = urdf_awesome_leg.q_max() - jnt_lim_margin_array

tau_lim = np.array([0, cs.inf, cs.inf])  # effort limits (also on the passive d.o.f.)

prb = Problem(n_nodes)  # initialization of a problem object

transcriptor_name = trans_name  
trans_opt = dict(integrator = trans_integrator)  # dictionary with the chosen integrator name

# Dt
dt1 = prb.createSingleVariable("dt1", 1)  # dt before the takeoff
dt2 = prb.createSingleVariable("dt2", 1)  # dt during flight
dt3 = prb.createSingleVariable("dt3", 1)  # dt after touchdown
dt1.setBounds(dt_lb, dt_ub)  # bounds on dt1
dt2.setBounds(dt_lb, dt_ub)  # bounds on dt2
dt3.setBounds(dt_lb, dt_ub)  # bounds on dt3

Dt=[dt1]*n_takeoff+[dt2]*(n_touchdown-n_takeoff)+[dt3]*(n_nodes-n_touchdown) # holds the complete time list

prb.setDt(Dt)

# Creating the state variables
q_p = prb.createStateVariable("q_p", n_q)
if employ_opt_init:
    q_p.setInitialGuess(loaded_sol["q_p"])

q_p_dot = prb.createStateVariable("q_p_dot",
                                  n_v)  # here q_p_dot is actually not the derivative of the lagrangian state vector
if employ_opt_init:
    q_p_dot.setInitialGuess(loaded_sol["q_p_dot"])

q_p.setBounds(lbs, ubs) # test rig excursion limits

q_p.setBounds(q_p_init, q_p_init, 0)  # imposing the initial conditions (q_p) on the first node ("0")
q_p_dot.setBounds(q_p_dot_init, q_p_dot_init, 0)  # zero initial "velocity"

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
contact_map = dict(tip = f_contact)  # creating a contact map for applying the input to the foot

##############################################

prb.setDynamics(xdot)  # setting the dynamics we are interested of in the problem object (xdot)
trscptr = transcriptor.Transcriptor.make_method(transcriptor_name, prb, trans_opt)  # setting the transcriptor

##############################################

## Obtaining some relevant quantities
tau = kin_dyn.InverseDynamics(urdf_awesome_leg, contact_map.keys(),casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED).call(q_p, q_p_dot, q_p_ddot, contact_map) # obtaining the joint efforts

# hip
fk_hip = cs.Function.deserialize(urdf_awesome_leg.fk("hip1_1"))  # deserializing
hip_position_initial = fk_hip(q = q_p_init)["ee_pos"]  # initial hip position (numerical)
hip_position = fk_hip(q = q_p)["ee_pos"]  # hip position (symbolic)

# hip vel
dfk_hip = cs.Function.deserialize(
    urdf_awesome_leg.frameVelocity("hip1_1", casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
v_hip = dfk_hip(q = q_p, qdot = q_p_dot)["ee_vel_linear"]  # foot velocity

# foot tip pos
fk_foot = cs.Function.deserialize(urdf_awesome_leg.fk("tip"))
foot_tip_position_init = fk_foot(q = q_p_init)["ee_pos"]  # foot initial position
foot_tip_position = fk_foot(q = q_p)["ee_pos"]  # foot position

# foot tip vel
dfk_foot = cs.Function.deserialize(
    urdf_awesome_leg.frameVelocity("tip", casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
v_foot_tip = dfk_foot(q=q_p, qdot=q_p_dot)["ee_vel_linear"]  # foot velocity

##############################################

## Constraints

tau_limits = prb.createIntermediateConstraint("tau_limits", tau)  # torque limits
tau_limits.setBounds(-tau_lim, tau_lim)  # setting input limits

prb.createConstraint("foot_vel_bf_touchdown", v_foot_tip,
                     nodes=range(0, n_takeoff + 1))  # no velocity of the foot before takeoff
prb.createConstraint("foot_vel_aftr_touchdown", v_foot_tip,
                     nodes=range(n_touchdown, n_nodes + 1))  # no velocity of the foot after touchdown

prb.createConstraint("GRF_zero", f_contact,
                     nodes=range(n_takeoff, n_touchdown))  # 0 GRF during flight

prb.createFinalConstraint("leg_pose_restoration", q_p - q_p_init)

prb.createFinalConstraint("final_joint_zero_vel", q_p_dot)  # joints are still at the end of the optimization horizon

# hip_over_foot_tip = prb.createConstraint("hip_over_foot_tip", hip_position[2] -  foot_tip_position[2]) # always keep the hip above the tip 
# hip_over_foot_tip.setBounds(0, cs.inf)

## Keep the ESTIMATED quadrature currents within bounds
# compensated_tau = []
# i_q_cnstr = []
# for i in range(n_q -1):
#     compensated_tau.append(tau[i + 1] + K_d0[i] * np.tanh( tanh_coeff * q_p_dot[i + 1]) + K_d1[i] * q_p_dot[i + 1])
#     i_q = (rotor_axial_MoI[i] * q_p_ddot[i + 1] / red_ratio[i] + compensated_tau[i] * red_ratio[i] / efficiency[i]) / K_t[i]
#     i_q_cnstr.append( prb.createIntermediateConstraint("quadrature_current" + actuator_names[i], i_q) )
#     i_q_cnstr[i].setBounds(-I_peak[i], I_peak[i])  # setting input limits

##############################################

## Costs
prb.createIntermediateCost("min_f_contact", weight_contact_cost * cs.sumsqr(f_contact))

prb.createIntermediateCost("min_q_ddot", weight_q_ddot * cs.sumsqr(
    q_p_ddot))  # minimizing the joint accelerations ("responsiveness" of the trajectory)

prb.createIntermediateCost("max_hip_height_jump", weight_hip_height_jump * cs.sumsqr(1 / (hip_position[2])),
                           nodes=range(n_takeoff, n_touchdown))

prb.createIntermediateCost("max_foot_tip_clearance", weight_tip_clearance * cs.sumsqr(1 / (foot_tip_position[2])),
                           nodes=range(n_takeoff, n_touchdown))

prb.createIntermediateCost("min_input_diff", weight_min_input_diff * cs.sumsqr(q_p_ddot - q_p_ddot.getVarOffset(-1)),nodes = range(1, n_nodes))  

##############################################

## Creating the solver and solving the problem
slvr = solver.Solver.make_solver(slvr_name, prb, slvr_opt) 
t = time.time()
slvr.solve()  # solving
solution_time = time.time() - t
print(f'solved in {solution_time} s')
solution = slvr.getSolutionDict() # extracting solution
cnstr_opt = slvr.getConstraintSolutionDict()
tau_sol = cnstr_opt["tau_limits"]

################### RESAMPLING (necessary because dt is variable) #####################

q_sym = cs.SX.sym('q', n_q)
q_dot_sym = cs.SX.sym('q_dot', n_v)
q_ddot_sym = cs.SX.sym('q_ddot', n_v)
x, x_dot = utils.double_integrator(q_sym, q_dot_sym, q_ddot_sym)

dae = {'x': x, 'p': q_ddot_sym, 'ode': x_dot, 'quad': 1}

dt_res = rospy.get_param("horizon/horizon_resampler/dt")
sol_contact_map = dict(tip = solution["f_contact"])  # creating a contact map for applying the input to the foot

p_res, v_res, a_res, sol_contact_map_res, tau_res = resampler_trajectory.resample_torques(solution["q_p"],
                                                                                              solution["q_p_dot"],
                                                                                              solution["q_p_ddot"],
                                                                                              slvr.getDt().flatten(),
                                                                                              dt_res,
                                                                                              dae, sol_contact_map,
                                                                                              urdf_awesome_leg,
                                                                                              casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)

################### POST PROCESSING AND SOLUTION STORAGE #####################

## Original solution
# Hip and knee quadrature current estimation
i_q_n_samples = len(solution["q_p_ddot"][0, :])
i_q = np.zeros((n_q, i_q_n_samples))
for i in range(n_q - 1):
    compensated_tau = tau_sol[i + 1, :] + K_d0[i] * np.tanh( tanh_coeff * solution["q_p_dot"][i + 1, 1:(i_q_n_samples + 1)]) + K_d1[i] * solution["q_p_dot"][i + 1, 1:(i_q_n_samples + 1)]
    i_q[i, :] = (rotor_axial_MoI[i] * solution["q_p_ddot"][i + 1, :] / red_ratio[i] + compensated_tau * red_ratio[i] / efficiency[i]) / K_t[i]

solution_GRF = solution["f_contact"]
solution_q_p = solution["q_p"]
solution_foot_tip_position = fk_foot(q = solution_q_p)["ee_pos"][2,:].toarray()  # foot position
solution_hip_position = fk_hip(q=solution_q_p)["ee_pos"][2,:].toarray()   # hip position
init_solution_foot_tip_position_aux = np.tile(foot_tip_position_init,(1, n_nodes + 1)) # auxiliary matrix to compute position excursion
init_solution_hip_position_aux = np.tile(hip_position_initial,(1, n_nodes + 1))
solution_v_foot_tip = dfk_foot(q=solution["q_p"], qdot=solution["q_p_dot"])["ee_vel_linear"]  # foot velocity
solution_v_foot_hip = dfk_hip(q=solution["q_p"], qdot=solution["q_p_dot"])["ee_vel_linear"]  # foot velocity

useful_solutions={"q_p":solution["q_p"],"q_p_dot":solution["q_p_dot"], "q_p_ddot":solution["q_p_ddot"],
                 "tau":cnstr_opt["tau_limits"], "f_contact":solution["f_contact"], "i_q":i_q, "dt_opt":slvr.getDt(),
                 "foot_tip_height":np.transpose(solution_foot_tip_position-init_solution_foot_tip_position_aux[2,:]), 
                 "hip_height":np.transpose(solution_hip_position-init_solution_hip_position_aux[2,:]), 
                 "tip_velocity":np.transpose(np.transpose(solution_v_foot_tip)),
                 "hip_velocity":np.transpose(np.transpose(solution_v_foot_hip)),
                 "sol_time":solution_time}

ms.store(useful_solutions) # saving solution data to file

## Resampled solution
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

ms_res.store(useful_solutions_res) # saving solution data to file

# copying stuff for future debugging
shutil.copyfile(opt_res_path + "/jump_test/horizon_offline_solver.mat", target + sol_mat_name + current_time + ".mat")
shutil.copyfile(opt_res_path + "/jump_test/horizon_offline_solver_res.mat", target + res_sol_mat_name + current_time + ".mat")

if save_sol_as_init: # save the solution as the initialization for the next sim
    ms_opt_init.store(useful_solutions) # saving initialization data to file    

################### REPLAYING TRAJECTORY ON RVIZ #####################

replay_traj = rospy.get_param("/horizon/replay_trajectory")

if replay_traj:
    joint_names = urdf_awesome_leg.joint_names()
    joint_names.remove("universe")  # removing the "universe joint"
    rpl_traj = replay_trajectory(dt_res, joint_names, p_res, sol_contact_map_res, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED, urdf_awesome_leg)  # replaying the (resampled) trajectory
    # rpl_traj = replay_trajectory(slvr.getDt()[0], joint_names, solution["q_p"])  # replaying the (resampled) trajectory
    rpl_traj.sleep(1.)
    rpl_traj.replay(is_floating_base = False)

