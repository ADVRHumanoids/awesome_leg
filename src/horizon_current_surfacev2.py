#!/usr/bin/env python3

###############################################

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

##############################################

rospackage=rospkg.RosPack()

# Only for taking the path to the leg package
ms = mat_storer.matStorer(rospackage.get_path("awesome_leg_pholus")+"/sim_results/horizon_offline_solver.mat")

# "Loading the "URDF"
urdf_path = rospackage.get_path("awesome_leg_pholus")+"/description/urdf/awesome_leg_complete.urdf"
urdf = open(urdf_path, "r").read()
urdf_awesome_leg = casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn(urdf)

n_q = urdf_awesome_leg.nq()  # number of joints
n_v = urdf_awesome_leg.nv()  # number of dofs

# Setting some of the problem's parameters

T_f = rospy.get_param("/horizon/problem/T_f_horizon")  # optimization horizon
T_takeoff =rospy.get_param("/horizon/problem/T_takeoff_horizon")   # instant of takeoff
T_touchdown =rospy.get_param("/horizon/problem/T_touchdown_horizon")   # instant of touchdown
dt = rospy.get_param("/horizon/problem/dt")   # optimization dt [s]

n_nodes = round(T_f / dt)
n_takeoff = round(T_takeoff / dt)  # node index at takeoff
n_touchdown = round(T_touchdown / dt)  # node index at touchdown

rospy.set_param('/horizon/problem/n_nodes', n_nodes) # setting ros parameters (might be useful for other nodes)
rospy.set_param('/horizon/problem/n_takeoff', n_takeoff)
rospy.set_param('/horizon/problem/n_touchdown', n_touchdown)

test_rig_lb= rospy.get_param("/horizon/test_rig/lb") # lower bound of the test rig excursion
test_rig_ub=rospy.get_param("/horizon/test_rig/ub") # upper bound of the test rig excursion
hip_tau_lim=rospy.get_param("/horizon/hip/tau_lim") # hip effort limit [Nm]
knee_tau_lim=rospy.get_param("/horizon/knee/tau_lim") # knee effort limit [Nm]
tau_lim = np.array([0, hip_tau_lim, knee_tau_lim])  # effort limits (test_rig passive joint effort limit)

# initial joint config (ideally it would be given from measurements)

q_p_init = rospy.get_param("/horizon/initial_conditions/q_p")
q_p_dot_init = rospy.get_param("/horizon/initial_conditions/q_p_dot")

# cost weights
weight_contact_cost = rospy.get_param("/horizon/cost_weights/contact_force")  # minimizing the contact force
weight_postural_cost = rospy.get_param("/horizon/cost_weights/terminal_postural") # cost to restore the initial position at the end of the control horizon
weight_q_ddot = rospy.get_param("/horizon/cost_weights/small_q_p_ddot")# minimizing joint accelerations
weight_hip_height_jump = rospy.get_param("/horizon/cost_weights/big_hip_jump") # maximizing the jump height (measured at the hip)

# solver options
slvr_opt = {"ipopt.tol": rospy.get_param("/horizon/problem/solver/tolerance"), "ipopt.max_iter": rospy.get_param("/horizon/problem/solver/max_iter"), "ipopt.linear_solver": rospy.get_param("/horizon/problem/solver/linear_solver_name")}
slvr_name=rospy.get_param("/horizon/problem/solver/name") # options: "blocksqp", "ipopt", "ilqr", "gnsqp", 

# hip actuator
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

# knee actuator
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

prb = Problem(n_nodes)  # initialization of a problem object
prb.setDt(dt)  # setting problem's dt

transcriptor_name = "multiple_shooting"  # other option: "direct_collocation"
trans_opt = dict(integrator="RK4")  # dictionary with the chosen integrator name

##############################################

# Creating the state variables
q_p = prb.createStateVariable("q_p", n_q)
q_p_dot = prb.createStateVariable("q_p_dot",
                                  n_v)  # here q_p_dot is actually not the derivative of the lagrangian state vector

q_p[0].setBounds(test_rig_lb, test_rig_ub) # test rig excursion
q_p.setBounds(q_p_init, q_p_init, 0)  # imposing the initial conditions (q_p) on the first node ("0")
q_p_dot.setBounds(q_p_dot_init, q_p_dot_init, 0)  # zero initial "velocity"

# Defining the input/s (joint accelerations)
q_p_ddot = prb.createInputVariable("q_p_ddot", n_v)  # using joint accelerations as an input variable
x, xdot = utils.double_integrator(q_p, q_p_dot, q_p_ddot)  # building the full state

# Creating an additional input variable for the contact forces on the foot tip
f_contact = prb.createInputVariable("f_contact", 3)  # dimension 3
f_contact[2].setInitialGuess(100)  # initial guess (set to leg's weight)
f_contact[2].setLowerBounds(0)  # the vertical component of f_contact needs to be always positive
contact_map = dict(tip=f_contact)  # creating a contact map for applying the input to the foot

##############################################

prb.setDynamics(xdot)  # setting the dynamics we are interested of in the problem object (xdot)
trscptr = transcriptor.Transcriptor.make_method(transcriptor_name, prb, trans_opt)  # setting the transcriptor

##############################################

# obtaining relevant quantities
tau = kin_dyn.InverseDynamics(urdf_awesome_leg, contact_map.keys(),casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED).call(q_p, q_p_dot,q_p_ddot,contact_map) # obtaining the joint efforts

fk_hip = cs.Function.deserialize(urdf_awesome_leg.fk("hip1_1"))  # deserializing
hip_position_initial = fk_hip(q=q_p_init)["ee_pos"]  # initial hip position (numerical)
hip_position = fk_hip(q=q_p)["ee_pos"]  # hip position (symbolic)

dfk_foot = cs.Function.deserialize(
    urdf_awesome_leg.frameVelocity("tip", casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
fk_foot = cs.Function.deserialize(urdf_awesome_leg.fk("tip"))
p_foot_tip_init = fk_foot(q=q_p_init)["ee_pos"]  # foot initial position
p_foot_tip = fk_foot(q=q_p)["ee_pos"]  # foot position
v_foot_tip = dfk_foot(q=q_p, qdot=q_p_dot)["ee_vel_linear"]  # foot velocity

##############################################

# constraints
tau_cnstrnt = prb.createIntermediateConstraint("dynamics_feas", tau)  # dynamics feasibility constraint
tau_cnstrnt.setBounds(-tau_lim, tau_lim)  # setting input limits

prb.createConstraint("foot_vel_bf_touchdown", v_foot_tip,
                     nodes=range(0, n_takeoff + 1))  # no vertical velocity of the foot before takeoff
prb.createConstraint("foot_vel_aftr_touchdown", v_foot_tip,
                     nodes=range(n_touchdown, n_nodes + 1))  # no vertical velocity of the foot after touchdown
prb.createConstraint("foot_pos_restoration", p_foot_tip - p_foot_tip_init,
                     nodes=n_nodes)  # restore foot position at the end of the optimization horizon
prb.createConstraint("GRF_zero", f_contact,
                     nodes=range(n_takeoff, n_touchdown))  # 0 GRF during flight
prb.createFinalConstraint("final_joint_zero_vel", q_p_dot)  # joints are still at the end of the optimization horizon


i_q_hip=prb.createConstraint("quadrature_current_hip", (hip_axial_MoI*q_p_ddot/hip_red_ratio+tau*hip_red_ratio/hip_efficiency)*1.0/hip_K_t)  # i_q hip less than the maximum allowed value
i_q_knee=prb.createConstraint("quadrature_current_knee", (knee_axial_MoI*q_p_ddot/knee_red_ratio+tau*knee_red_ratio/knee_efficiency)*1.0/knee_K_t)  # i_q knee less than the maximum allowed value
# i_q_hip.setBounds(-np.ones(n_nodes-1)*hip_I_max, np.ones(n_nodes-1)*hip_I_max)  # setting input limits
# i_q_knee.setBounds(-np.ones(n_nodes-1)*knee_I_max, np.ones(n_nodes-1)*knee_I_max)  # setting input limits
##############################################

# costs 
prb.createIntermediateCost("min_f_contact", weight_contact_cost * cs.sumsqr(f_contact))
prb.createIntermediateCost("min_q_ddot", weight_q_ddot * cs.sumsqr(
    q_p_ddot))  # minimizing the joint accelerations ("responsiveness" of the trajectory)
prb.createFinalCost("postural", weight_postural_cost * cs.sumsqr(
    q_p - q_p_init))  # penalizing the difference between the initial position and the final one (using it as a constraint does not work)
prb.createIntermediateCost("max_hip_height_jump", weight_hip_height_jump * cs.sumsqr(1 / (hip_position[2])),
                           nodes=range(n_takeoff, n_touchdown))

##############################################

# creating the solver and solving the problem
slvr = solver.Solver.make_solver(slvr_name, prb, slvr_opt) 
t = time.time()
slvr.solve()  # solving
solution_time = time.time() - t
print(f'solved in {solution_time} s')

solution = slvr.getSolutionDict() # extracting solution
# dt_opt = slvr.getDt() # extracting dt --> returns an array with the dt for each node

joint_names = urdf_awesome_leg.joint_names()
joint_names.remove("universe")  # removing the "universe joint"

cnstr_opt = slvr.getConstraintSolutionDict()

solution_GRF = solution["f_contact"]
solution_q_p = solution["q_p"]
solution_p_foot_tip = fk_foot(q=solution_q_p)["ee_pos"]  # foot position
solution_hip_position = fk_hip(q=solution_q_p)["ee_pos"]  # hip position

useful_solutions={"q_p":solution["q_p"][1:3,:],"q_p_dot":solution["q_p_dot"][1:3,:], "q_p_ddot":solution["q_p_ddot"][1:3,:], "tau":cnstr_opt["dynamics_feas"][1:3,:], "f_contact":solution["f_contact"], "sol_time":solution_time}

ms.store(useful_solutions) # saving solution data to file

# rpl_traj = replay_trajectory(dt, joint_names,
#                              solution["q_p"])  # replaying the trajectory and the forces on (it publishes on ROS topics)
# rpl_traj.sleep(1.0)
# rpl_traj.replay(is_floating_base=False)

print(time_vector)