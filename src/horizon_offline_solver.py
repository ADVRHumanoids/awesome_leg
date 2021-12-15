#!/usr/bin/env python3

##################### IMPORTS #########################
import time

import rospy
import rospkg

from horizon.problem import Problem
from horizon.utils import kin_dyn, utils, mat_storer
from horizon.transcriptions import transcriptor
from horizon.solvers import solver
import casadi as cs
import casadi_kin_dyn

import numpy as np

##################### Reading the solution modes from config file #########################

is_adaptive_dt = rospy.get_param("/horizon_solver/is_adaptive_dt")  # if true, use an adaptive dt
save_sol_as_init = rospy.get_param("/horizon_solver/save_sol_as_init")  # if true, the solution is also saved as a candidate for future optimization initializations
employ_opt_init = rospy.get_param("/horizon_solver/employ_opt_init")  # if true, the solution is also saved as a candidate for future optimization initializations
is_single_dt = rospy.get_param("/horizon_solver/variable_dt/problem_settings/is_single_dt")  # if true (and if addaptive dt is enable), use only one dt over the entire opt. horizon 

##################### Initializing object for .mat storage #########################

rospackage=rospkg.RosPack() # Only for taking the path to the leg package

if employ_opt_init:
    ms_load = mat_storer.matStorer(rospackage.get_path("awesome_leg_pholus")+"/sim_results/horizon_offline_solver_init.mat")
    loaded_sol=ms_load.load() # loading the solution dictionary

ms = mat_storer.matStorer(rospackage.get_path("awesome_leg_pholus")+"/sim_results/horizon_offline_solver.mat")
if save_sol_as_init:
    ms_opt_init = mat_storer.matStorer(rospackage.get_path("awesome_leg_pholus")+"/sim_results/horizon_offline_solver_init.mat")

##################### LOADING SOLVER PARAMETERS FROM SERVER #########################


# Loading some problem paramters from the ROS parameter server
if is_adaptive_dt:

    n_nodes = rospy.get_param("/horizon_solver/variable_dt/problem_settings/n_nodes")  # optimization horizon
    n_takeoff =rospy.get_param("/horizon_solver/variable_dt/problem_settings/n_takeoff")   # instant of takeoff
    n_touchdown =rospy.get_param("/horizon_solver/variable_dt/problem_settings/n_touchdown")   # instant of touchdown
    n_pre_takeoff =rospy.get_param("/horizon_solver/variable_dt/problem_settings/n_pre_takeoff")   # instant of takeoff
    n_post_touchdown =rospy.get_param("/horizon_solver/variable_dt/problem_settings/n_post_touchdown")   # instant of touchdown
    dt_lb=rospy.get_param("/horizon_solver/variable_dt/problem_settings/dt_lb")   # dt lower bound 
    dt_ub=rospy.get_param("/horizon_solver/variable_dt/problem_settings/dt_ub")   # dt upper bound
    test_rig_lb= rospy.get_param("horizon_solver/variable_dt/problem_settings/test_rig/lb") # lower bound of the test rig excursion
    test_rig_ub=rospy.get_param("/horizon_solver/variable_dt/problem_settings/test_rig/ub") # upper bound of the test rig excursion
    # hip_lb= rospy.get_param("/horizon_solver/variable_dt/problem_settings/hip/lb") # lower bound of the hip joint
    # hip_ub=rospy.get_param("/horizon_solver/variable_dt/problem_settings/hip/ub") # upper bound of the hip joint
    # top_lb= rospy.get_param("/horizon_solver/variable_dt/problem_settings/knee/lb") # lower bound of the hip joint
    # top_ub=rospy.get_param("/horizon_solver/variable_dt/problem_settings/knee/ub") # upper bound of the hip joint
    q_p_init = rospy.get_param("/horizon_solver/variable_dt/problem_settings/initial_conditions/q_p") # initial joint config (ideally it would be given from measurements)
    q_p_dot_init = rospy.get_param("/horizon_solver/variable_dt/problem_settings/initial_conditions/q_p_dot") # initial joint config (ideally it would be given from measurements)

    # cost weights

    weight_contact_cost = rospy.get_param("/horizon_solver/variable_dt/problem_settings/cost_weights/contact_force")  # minimizing the contact force
    weight_postural_cost = rospy.get_param("/horizon_solver/variable_dt/problem_settings/cost_weights/terminal_postural") # cost to restore the initial position at the end of the control horizon
    weight_q_ddot = rospy.get_param("/horizon_solver/variable_dt/problem_settings/cost_weights/small_q_p_ddot")# minimizing joint accelerations
    weight_hip_height_jump = rospy.get_param("/horizon_solver/variable_dt/problem_settings/cost_weights/big_hip_jump") # maximizing the jump height (measured at the hip)
    weight_tip_clearance=rospy.get_param("/horizon_solver/variable_dt/problem_settings/cost_weights/big_foot_tip_clearance") # maximizing the jump height (measured at the tip)
    
    # solver options

    slvr_opt = {"ipopt.tol": rospy.get_param("/horizon_solver/variable_dt/problem_settings/solver/tolerance"), "ipopt.max_iter": rospy.get_param("/horizon_solver/variable_dt/problem_settings/solver/max_iter"), "ipopt.linear_solver": rospy.get_param("/horizon_solver/variable_dt/problem_settings/solver/linear_solver_name")} 
    slvr_name=rospy.get_param("/horizon_solver/variable_dt/problem_settings/solver/name") # options: "blocksqp", "ipopt", "ilqr", "gnsqp", 

else:

    dt=rospy.get_param("/horizon_solver/constant_dt/problem_settings/dt")
    T_f = rospy.get_param("/horizon_solver/constant_dt/problem_settings/T_f_horizon")  # optimization horizon
    T_takeoff =rospy.get_param("/horizon_solver/constant_dt/problem_settings/T_takeoff_horizon")   # instant of takeoff
    T_touchdown =rospy.get_param("/horizon_solver/constant_dt/problem_settings/T_touchdown_horizon")   # instant of touchdown
    n_nodes = round(T_f / dt)
    n_takeoff = round(T_takeoff / dt)  # node index at takeoff
    n_touchdown = round(T_touchdown / dt)  # node index at touchdown
    rospy.set_param('/horizon_solver/constant_dt/problem_settings/n_nodes', n_nodes) # setting ros parameters (might be useful for other nodes)
    rospy.set_param('/horizon_solver/constant_dt/problem_settings/n_takeoff', n_takeoff)
    rospy.set_param('/horizon_solver/constant_dt/problem_settings/n_touchdown', n_touchdown)
    test_rig_lb= rospy.get_param("/horizon_solver/constant_dt/problem_settings/test_rig/lb") # lower bound of the test rig excursion
    test_rig_ub=rospy.get_param("/horizon_solver/constant_dt/problem_settings/test_rig/ub") # upper bound of the test rig excursion
    # hip_lb= rospy.get_param("/horizon_solver/constant_dt/problem_settings/hip/lb") # lower bound of the hip joint
    # hip_ub=rospy.get_param("/horizon_solver/constant_dt/problem_settings/hip/ub") # upper bound of the hip joint
    # top_lb= rospy.get_param("/horizon_solver/constant_dt/problem_settings/knee/lb") # lower bound of the hip joint
    # top_ub=rospy.get_param("/horizon_solver/constant_dt/problem_settings/knee/ub") # upper bound of the hip joint
    q_p_init = rospy.get_param("/horizon_solver/constant_dt/problem_settings/initial_conditions/q_p") # initial joint config (ideally it would be given from measurements)
    q_p_dot_init = rospy.get_param("/horizon_solver/constant_dt/problem_settings/initial_conditions/q_p_dot") # initial joint config (ideally it would be given from measurements)

    # cost weights

    weight_contact_cost = rospy.get_param("/horizon_solver/constant_dt/problem_settings/cost_weights/contact_force")  # minimizing the contact force
    weight_postural_cost = rospy.get_param("/horizon_solver/constant_dt/problem_settings/cost_weights/terminal_postural") # cost to restore the initial position at the end of the control horizon
    weight_q_ddot = rospy.get_param("/horizon_solver/constant_dt/problem_settings/cost_weights/small_q_p_ddot")# minimizing joint accelerations
    weight_hip_height_jump = rospy.get_param("/horizon_solver/constant_dt/problem_settings/cost_weights/big_hip_jump") # maximizing the jump height (measured at the hip)
    weight_tip_clearance=rospy.get_param("/horizon_solver/constant_dt/problem_settings/cost_weights/big_foot_tip_clearance") # maximizing the jump height (measured at the tip)
    # solver options

    slvr_opt = {"ipopt.tol": rospy.get_param("/horizon_solver/constant_dt/problem_settings/solver/tolerance"), "ipopt.max_iter": rospy.get_param("/horizon_solver/constant_dt/problem_settings/solver/max_iter"), "ipopt.linear_solver": rospy.get_param("/horizon_solver/constant_dt/problem_settings/solver/linear_solver_name")}
    slvr_name=rospy.get_param("/horizon_solver/constant_dt/problem_settings/solver/name") # options: "blocksqp", "ipopt", "ilqr", "gnsqp", 



###############################################

## Getting the properties of the actuator from the ROS parameter server

# hip actuator:
hip_axial_MoI=rospy.get_param("/actuators/hip/axial_MoI")
hip_mass=rospy.get_param("/actuators/hip/mass")
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

# knee actuator:
knee_axial_MoI=rospy.get_param("/actuators/knee/axial_MoI")
knee_mass=rospy.get_param("/actuators/knee/mass")
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

##############################################

# Loading the URDF
urdf_path = rospackage.get_path("awesome_leg_pholus")+"/description/urdf/awesome_leg_complete.urdf"
urdf = open(urdf_path, "r").read()
urdf_awesome_leg = casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn(urdf)

n_q = urdf_awesome_leg.nq()  # number of joints
n_v = urdf_awesome_leg.nv()  # number of dofs

##############################################

tau_lim = np.array([0, hip_tau_max_ar, knee_tau_max_ar])  # effort limits (test_rig passive joint effort limit)

prb = Problem(n_nodes)  # initialization of a problem object

transcriptor_name = "multiple_shooting"  # other option: "direct_collocation"
trans_opt = dict(integrator="RK4")  # dictionary with the chosen integrator name

##############################################

if is_adaptive_dt:

    if is_single_dt:

        Dt = prb.createSingleVariable("dt", 1)  # dt before the takeoff
        Dt.setBounds(dt_lb, dt_ub)  # bounds on dt
        
    else:
        dt1 = prb.createSingleVariable("dt1", 1)  # dt before the takeoff
        dt2 = prb.createSingleVariable("dt2", 1)  # dt during flight
        dt3 = prb.createSingleVariable("dt3", 1)  # dt after touchdown
        dt2_bis = prb.createSingleVariable("dt2_bis", 1)  # dt during flight
        dt3_bis = prb.createSingleVariable("dt3_bis", 1)  # dt after touchdown

        dt1.setBounds(dt_lb, dt_ub)  # bounds on dt1
        dt2.setBounds(dt_lb, dt_ub)  # bounds on dt2
        dt3.setBounds(dt_lb, dt_ub)  # bounds on dt3
        dt2_bis.setBounds(dt_lb, dt_ub)  # bounds on dt2
        dt3_bis.setBounds(dt_lb, dt_ub)  # bounds on dt3

        Dt=[dt1]*n_pre_takeoff+[dt2_bis]*(n_takeoff-n_pre_takeoff)+[dt2]*(n_touchdown-n_takeoff)+[dt3_bis]*(n_post_touchdown-n_touchdown)+[dt3]*(n_nodes-n_post_touchdown) # holds the complete time list
    
    prb.setDt(Dt)
     
else:
    prb.setDt(dt)

# Creating the state variables
q_p = prb.createStateVariable("q_p", n_q)
if employ_opt_init:
    q_p[1:].setInitialGuess(loaded_sol["q_p"])

q_p_dot = prb.createStateVariable("q_p_dot",
                                  n_v)  # here q_p_dot is actually not the derivative of the lagrangian state vector
if employ_opt_init:
    q_p_dot[1:].setInitialGuess(loaded_sol["q_p_dot"])

q_p[0].setBounds(test_rig_lb, test_rig_ub) # test rig excursion
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
contact_map = dict(tip=f_contact)  # creating a contact map for applying the input to the foot

##############################################

prb.setDynamics(xdot)  # setting the dynamics we are interested of in the problem object (xdot)
trscptr = transcriptor.Transcriptor.make_method(transcriptor_name, prb, trans_opt)  # setting the transcriptor

##############################################

## Obtaining some relevant quantities
tau = kin_dyn.InverseDynamics(urdf_awesome_leg, contact_map.keys(),casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED).call(q_p, q_p_dot,q_p_ddot,contact_map) # obtaining the joint efforts

# hip
fk_hip = cs.Function.deserialize(urdf_awesome_leg.fk("hip1_1"))  # deserializing
hip_position_initial = fk_hip(q=q_p_init)["ee_pos"]  # initial hip position (numerical)
hip_position = fk_hip(q=q_p)["ee_pos"]  # hip position (symbolic)

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
tau_cnstrnt = prb.createIntermediateConstraint("dynamics_feas", tau)  # dynamics feasibility constraint
tau_cnstrnt.setBounds(-tau_lim, tau_lim)  # setting input limits

prb.createConstraint("foot_vel_bf_touchdown", v_foot_tip,
                     nodes=range(0, n_takeoff + 1))  # no vertical velocity of the foot before takeoff
prb.createConstraint("foot_vel_aftr_touchdown", v_foot_tip,
                     nodes=range(n_touchdown, n_nodes + 1))  # no vertical velocity of the foot after touchdown
prb.createConstraint("foot_pos_restoration", foot_tip_position - foot_tip_position_init,
                     nodes=n_nodes)  # restore foot position at the end of the optimization horizon
prb.createConstraint("GRF_zero", f_contact,
                     nodes=range(n_takeoff, n_touchdown))  # 0 GRF during flight
prb.createFinalConstraint("final_joint_zero_vel", q_p_dot)  # joints are still at the end of the optimization horizon

i_q_hip=prb.createIntermediateConstraint("quadrature_current_hip", (hip_axial_MoI*q_p_ddot[1]/hip_red_ratio+tau[1]*hip_red_ratio/hip_efficiency)*1.0/hip_K_t)  # i_q hip less than the maximum allowed value

i_q_knee=prb.createIntermediateConstraint("quadrature_current_knee", (knee_axial_MoI*q_p_ddot[2]/knee_red_ratio+tau[2]*knee_red_ratio/knee_efficiency)*1.0/knee_K_t)  # i_q knee less than the maximum allowed value

i_q_hip.setBounds(-hip_I_peak, hip_I_peak)  # setting input limits
i_q_knee.setBounds(-knee_I_peak, knee_I_peak)  # setting input limits

##############################################

## Costs
 
prb.createIntermediateCost("min_f_contact", weight_contact_cost * cs.sumsqr(f_contact))
prb.createIntermediateCost("min_q_ddot", weight_q_ddot * cs.sumsqr(
    q_p_ddot))  # minimizing the joint accelerations ("responsiveness" of the trajectory)
prb.createFinalCost("postural", weight_postural_cost * cs.sumsqr(
    q_p - q_p_init))  # penalizing the difference between the initial position and the final one (using it as a constraint does not work)
prb.createIntermediateCost("max_hip_height_jump", weight_hip_height_jump * cs.sumsqr(1 / (hip_position[2])),
                           nodes=range(n_takeoff, n_touchdown))
prb.createIntermediateCost("max_foot_tip_clearance", weight_tip_clearance * cs.sumsqr(1 / (foot_tip_position[2])),
                           nodes=range(n_takeoff, n_touchdown))
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

i_q=(hip_axial_MoI*solution["q_p_ddot"][1:3,:]/hip_red_ratio+cnstr_opt["dynamics_feas"][1:3,:]*hip_red_ratio/hip_efficiency)*1.0/hip_K_t

solution_GRF = solution["f_contact"]
solution_q_p = solution["q_p"]
solution_foot_tip_position = fk_foot(q=solution_q_p)["ee_pos"]  # foot position
solution_hip_position = fk_hip(q=solution_q_p)["ee_pos"]  # hip position

useful_solutions={"q_p":solution["q_p"][1:3,:],"q_p_dot":solution["q_p_dot"][1:3,:], "q_p_ddot":solution["q_p_ddot"][1:3,:], "tau":cnstr_opt["dynamics_feas"][1:3,:], "f_contact":solution["f_contact"], "i_q":i_q, "dt_opt":slvr.getDt(), "sol_time":solution_time}

ms.store(useful_solutions) # saving solution data to file
if save_sol_as_init:
    ms_opt_init.store(useful_solutions) # saving initialization data to file
