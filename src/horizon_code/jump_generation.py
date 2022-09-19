#!/usr/bin/env python3

from fileinput import filename
import os as scibidibi
import shutil
import time
from datetime import date

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

from datetime import datetime

import subprocess

file_name = scibidibi.path.splitext(scibidibi.path.basename(__file__))[0]
file_name.replace(".py", "")

## Getting/setting some useful variables
today = date.today()
today_is = today.strftime("%d_%m_%Y")
now = datetime.now()
current_time = now.strftime("%H_%M_%S")
unique_id = today_is + "-" + current_time

## Reading the solution modes from config YAML config file

# if true, the solution is also saved as a candidate for future optimization initializations
is_refine_phase = rospy.get_param("horizon/horizon_solver/is_refine_phase")  
# if true, the solution is also saved as a candidate for future optimization initializations

opt_init_name = rospy.get_param("horizon/horizon_solver/opt_init_name") # name of the initialization file to be loaded

xacro_path = rospy.get_param("/horizon/xacro_path")
urdf_path = rospy.get_param("/horizon/urdf_path")  
urdf_name = rospy.get_param("/horizon/urdf_name")  
is_calibrated = rospy.get_param("/horizon/is_calibrated")  

opt_res_path = rospy.get_param("/horizon/opt_results_path")  # urdf relative path (wrt to the package)
refine_path = rospy.get_param("/horizon/refine_path")  # urdf relative path (wrt to the package)

tanh_coeff = rospy.get_param("horizon/horizon_i_q_estimator/tanh_coeff")  
# coefficient used by the approximated sign function ( sign = atanh(tanh_coeff * x) )

resample_traj = rospy.get_param("/horizon/resample_traj")

## Generate urdf

try:

    is_calibrated_command  = "calibrated_urdf:=" + str(is_calibrated).lower()

    xacro_gen = subprocess.check_call(["xacro",\
                                xacro_path, \
                                is_calibrated_command, \
                                "-o", 
                                urdf_path])
            
except:

    print('Failed to generate URDF.')


## problem parameters, based on the chosen options

n_int = rospy.get_param("horizon/horizon_solver/problem_settings/n_int")  # optimization horizon
n_takeoff =rospy.get_param("horizon/horizon_solver/problem_settings/n_takeoff")   # instant of takeoff
n_touchdown =rospy.get_param("horizon/horizon_solver/problem_settings/n_touchdown")   # instant of touchdown

dt_lb=rospy.get_param("horizon/horizon_solver/problem_settings/dt_lb")   # dt lower bound 
dt_ub=rospy.get_param("horizon/horizon_solver/problem_settings/dt_ub")   # dt upper bound
q_p_init = rospy.get_param("horizon/horizon_solver/problem_settings/initial_conditions/q_p") 
# initial joint config (ideally it would be given from measurements)

q_p_dot_init = rospy.get_param("horizon/horizon_solver/problem_settings/initial_conditions/q_p_dot") 
# initial joint config (ideally it would be given from measurements)

jnt_limit_margin = abs(rospy.get_param("horizon/horizon_solver/problem_settings/jnt_limit_margin")) 
# margin to be added to joint limits 

jnt_vel_limit_margin = abs(rospy.get_param("horizon/horizon_solver/problem_settings/jnt_vel_limit_margin")) 
# margin to be added to joint velocity limits 

## solver options
linear_solver_name= rospy.get_param("horizon/horizon_solver/problem_settings/solver/linear_solver_name")

# slvr_opt = {"ipopt.tol": rospy.get_param("horizon/horizon_solver/problem_settings/solver/tolerance"),
#     "ipopt.max_iter": rospy.get_param("horizon/horizon_solver/problem_settings/solver/max_iter"),
#     "ipopt.constr_viol_tol": rospy.get_param("horizon/horizon_solver/problem_settings/solver/cnstrnt_tolerance"), 
#     "ipopt.linear_solver": linear_solver_name} 

slvr_opt = {"ipopt.tol": rospy.get_param("horizon/horizon_solver/problem_settings/solver/tolerance"),
    "ipopt.max_iter": rospy.get_param("horizon/horizon_solver/problem_settings/solver/max_iter"),
    "ipopt.constr_viol_tol": rospy.get_param("horizon/horizon_solver/problem_settings/solver/cnstrnt_tolerance")}

slvr_name=rospy.get_param("horizon/horizon_solver/problem_settings/solver/name") # options: "blocksqp", "ipopt", "ilqr", "gnsqp", 

trans_name = rospy.get_param("horizon/horizon_solver/problem_settings/transcription/name")
trans_integrator = rospy.get_param("horizon/horizon_solver/problem_settings/transcription/integrator")

rospackage=rospkg.RosPack() # Only for taking the path to the leg package
config_path=rospackage.get_path("awesome_leg")+"/config/" # configuration files path
horizon_config_path = config_path + "horizon/"

## Creating folders for saving plots and other data (if not already existing). This folders are also used by horizon_plot.py

solution_mat_name = ""
results_dir_path = ""
loaded_sol = None
sol_mat_name = rospy.get_param("/horizon/horizon_solver/sol_mat_name")
if is_refine_phase: # initialize variables with the previously saved solution
    
    ms_load_path = refine_path + "/" + sol_mat_name + ".mat"
    ms_load = mat_storer.matStorer(ms_load_path)

    loaded_sol=ms_load.load() # loading the solution dictionary

    takeoff_node = np.max(np.where(loaded_sol["dt_opt"] == loaded_sol["dt1"])) + 1 # takeoff node index (0-based)
    touchdown_node = np.max(np.where(loaded_sol["dt_opt"] == loaded_sol["dt2"])) + 1 # takeoff node index (0-based)

    dt_res = loaded_sol["dt_opt_res"].flatten()[0]

    t_jump = np.sum(loaded_sol["dt_opt"].flatten()[0:takeoff_node])
    t_touchdown = np.sum(loaded_sol["dt_opt"].flatten()[0:touchdown_node])

    t_exec = sum(loaded_sol["dt_opt"].flatten())

    n_takeoff = int(np.round(t_jump / dt_res))
    n_touchdown = int(np.round(t_touchdown / dt_res))
 
    n_int = int(np.round(t_exec/dt_res))

    unique_id = str(loaded_sol["unique_id"][0]) # overrriding unique id so that the refined trajectory has the same name of its initialization

    sol_mat_name = rospy.get_param("/horizon/horizon_solver/ref_sol_mat_name")

    results_dir_path = opt_res_path + "/" + unique_id

else:

    results_dir_path = opt_res_path + "/" + unique_id

    if  (not scibidibi.path.isdir(opt_res_path)):

        scibidibi.makedirs(opt_res_path)
        
    if (not scibidibi.path.isdir(results_dir_path)):

        scibidibi.makedirs(results_dir_path)

        # saving config files for reference and future debugging

    shutil.copyfile(config_path + "actuators.yaml", results_dir_path + "/actuators_" + current_time + ".yaml") 
    shutil.copyfile(horizon_config_path + file_name + ".yaml", results_dir_path + "/" + file_name + "_" + current_time + ".yaml")
    shutil.copyfile(urdf_path, results_dir_path + "/" + urdf_name + "_" + current_time + ".urdf")


ms = mat_storer.matStorer(results_dir_path + "/" + sol_mat_name + ".mat")  # original opt. sol

## Actuator properties (see actuators.yaml for a brief description of most of the parameters)

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

I_lim = I_peak # limits set to the peak current 

## setting up problem 

urdf = open(urdf_path, "r").read()
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

# Probelm's dt
if is_refine_phase:

    dt = abs(rospy.get_param("horizon/horizon_solver/problem_settings/traj_refiner/dt_ref"))

    prb.setDt(dt)

else:

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
if is_refine_phase:
    q_p.setInitialGuess(loaded_sol["q_p_res"])

q_p_dot = prb.createStateVariable("q_p_dot",
                                  n_v)  # here q_p_dot is actually not the derivative of the lagrangian state vector
if is_refine_phase:
    q_p_dot.setInitialGuess(loaded_sol["q_p_dot_res"])

q_p.setBounds(lbs, ubs) 
q_p_dot[1:3].setBounds(- v_bounds, v_bounds)

if is_refine_phase:

    q_p.setBounds(loaded_sol["q_p"][:, 0], loaded_sol["q_p"][:, 0], 0)  
    q_p_dot.setBounds(loaded_sol["q_p_dot"][:, 0], loaded_sol["q_p_dot"][:, 0], 0) 

else:
        
    q_p.setBounds(q_p_init, q_p_init, 0)  # imposing the initial conditions (q_p) on the first node ("0")
    q_p_dot.setBounds(q_p_dot_init, q_p_dot_init, 0)  # zero initial "velocity"

# Defining the input/s (joint accelerations)
q_p_ddot = prb.createInputVariable("q_p_ddot", n_v)  # using joint accelerations as an input variable
if is_refine_phase:
    q_p_ddot.setInitialGuess(loaded_sol["q_p_ddot_res"])

x, xdot = utils.double_integrator(q_p, q_p_dot, q_p_ddot)  # building the full state

# Creating an additional input variable for the contact forces on the foot tip
f_contact = prb.createInputVariable("f_contact", 3)  # dimension 3
if is_refine_phase:
    f_contact.setInitialGuess(loaded_sol["f_contact_res"])
else:
    f_contact[2].setInitialGuess(100.0)

f_contact[2].setLowerBounds(0)  # the vertical component of f_contact needs to be always positive
contact_map = dict(tip1 = f_contact)  # creating a contact map for applying the input to the foot


prb.setDynamics(xdot)  # setting the dynamics we are interested of in the problem object (xdot)

trscptr = transcriptor.Transcriptor.make_method(transcriptor_name, prb, trans_opt)  # setting the transcriptor

tau = kin_dyn.InverseDynamics(urdf_awesome_leg, contact_map.keys(), \
        casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED).call(q_p, q_p_dot, q_p_ddot, contact_map) 

# hip
fk_hip = cs.Function.deserialize(urdf_awesome_leg.fk("base_link"))  # deserializing
hip_position_initial = fk_hip(q = q_p_init)["ee_pos"]  # initial hip position (numerical)
hip_position = fk_hip(q = q_p)["ee_pos"]  # hip position (symbolic)

# hip vel
dfk_hip = cs.Function.deserialize(
    urdf_awesome_leg.frameVelocity("base_link",\
            casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
v_hip = dfk_hip(q = q_p, qdot = q_p_dot)["ee_vel_linear"]  # foot velocity

# foot tip pos
fk_foot = cs.Function.deserialize(urdf_awesome_leg.fk("tip1"))
foot_tip_position_init = fk_foot(q = q_p_init)["ee_pos"]  # foot initial position
foot_tip_position = fk_foot(q = q_p)["ee_pos"]  # foot position

# foot tip vel
dfk_foot = cs.Function.deserialize(
    urdf_awesome_leg.frameVelocity("tip1",\
        casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
v_foot_tip = dfk_foot(q=q_p, qdot=q_p_dot)["ee_vel_linear"]  # foot velocity

# center of mass
com_f = cs.Function.deserialize(urdf_awesome_leg.centerOfMass())
com = com_f(q = q_p, v = q_p_dot, a = q_p_ddot)["com"]

## Constraints
tau_limits = prb.createIntermediateConstraint("tau_limits", tau)  # torque limits
tau_limits.setBounds(-tau_lim, tau_lim)  # setting input limits

prb.createConstraint("foot_vel_bf_touchdown", v_foot_tip,
                     nodes=range(0, n_takeoff))  # no velocity of the foot before takeoff
prb.createConstraint("foot_vel_aftr_touchdown", v_foot_tip,
                     nodes=range(n_touchdown, n_int + 1))  # no velocity of the foot after touchdown

prb.createConstraint("GRF_zero", f_contact,
                     nodes=range(n_takeoff, n_touchdown))  # 0 GRF during flight

prb.createFinalConstraint("leg_pose_restoration", q_p - q_p_init)

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
mu_friction_cone = abs(rospy.get_param("horizon/horizon_solver/problem_settings/mu_friction_cone"))

# friction_cone_bf_takeoff = prb.createConstraint("friction_cone_bf_takeoff",\
#                                     (f_contact[0]**2 + f_contact[1]**2) - (mu_friction_cone * f_contact[2])**2,
#                      nodes=range(0, n_takeoff))
# friction_cone_bf_takeoff.setBounds(-cs.inf, 0)

# friction_cone_aftr_touchdown = prb.createConstraint("friction_cone_aftr_touchdown",\
#                                     (f_contact[0]**2 + f_contact[1]**2) - (mu_friction_cone * f_contact[2])**2,
#                      nodes=range(n_touchdown, n_int)) # f_contact not active on last node
# friction_cone_aftr_touchdown.setBounds(-cs.inf, 0)

# quadratization of the linearized friction cone (easier to write, but quadratic)
# friction_cone_bf_takeoff = prb.createConstraint("friction_cone_bf_takeoff",\
#                                     f_contact[1] - (mu_friction_cone * f_contact[2]),
#                      nodes=range(0, n_takeoff))
# friction_cone_bf_takeoff.setBo2unds(-cs.inf, 0)

# friction_cone_aftr_touchdown = prb.createConstraint("friction_cone_aftr_touchdown",\
#                                     (f_contact[1])**2 - (mu_friction_cone * f_contact[2])**2,
#                      nodes=range(n_touchdown, n_int)) # f_contact not active on last node
# friction_cone_aftr_touchdown.setBounds(-cs.inf, 0)

# linearized friction cone (constraint need to be split in two because setBounds only takes numerical vals)

friction_cone_bf_takeoff1 = prb.createConstraint("friction_cone_bf_takeoff1",\
                                    f_contact[1] - (mu_friction_cone * f_contact[2]),
                     nodes=range(0, n_takeoff))
friction_cone_bf_takeoff1.setBounds(-cs.inf, 0)
friction_cone_bf_takeoff2 = prb.createConstraint("friction_cone_bf_takeoff2",\
                                    f_contact[1] + (mu_friction_cone * f_contact[2]),
                     nodes=range(0, n_takeoff))
friction_cone_bf_takeoff2.setBounds(0, cs.inf)

friction_cone_aftr_touchdown1 = prb.createConstraint("friction_cone_aftr_touchdown1",\
                                    f_contact[1] - mu_friction_cone * f_contact[2],
                     nodes=range(n_touchdown, n_int)) # f_contact not active on last node
friction_cone_aftr_touchdown1.setBounds(-cs.inf, 0)
friction_cone_aftr_touchdown2 = prb.createConstraint("friction_cone_aftr_touchdown2",\
                                    f_contact[1] + mu_friction_cone * f_contact[2],
                     nodes=range(n_touchdown, n_int)) # f_contact not active on last node
friction_cone_aftr_touchdown2.setBounds(0, cs.inf)

## costs
epsi = 1

scale_factor_base = rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/scale_factor_costs_base")   

cost_scaling_factor = dt_lb * n_int * scale_factor_base

weight_init_sol_tracking = rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/traj_refiner/weight_init_sol_tracking")

if is_refine_phase: # cost functions for the refinement phase
    
    weight_min_input_diff = rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/traj_refiner/weight_min_input_diff") 
    weight_min_f_contact_diff = rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/traj_refiner/weight_min_f_contact_diff")
    weight_contact_cost = rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/traj_refiner/contact_force")  # minimizing the contact force
    weight_q_ddot = rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/traj_refiner/small_q_p_ddot")# minimizing joint accelerations
    weight_hip_height_jump = rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/traj_refiner/big_hip_jump") 
    weight_tip_clearance = rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/traj_refiner/big_foot_tip_clearance") 
    weight_com_height = rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/traj_refiner/weight_com_height")

    weight_init_sol_tracking = weight_init_sol_tracking / cost_scaling_factor

    for i in range(n_int):

        prb.createIntermediateCost("init_sol_tracking_cost_n" + str(i + 1), weight_init_sol_tracking * cs.sumsqr(q_p - loaded_sol["q_p_res"][:, i + 1]), 
                                    nodes= i + 1) 

else: # cost functions for the original problem
    
    weight_min_input_diff = rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/weight_min_input_diff") 
    weight_min_f_contact_diff = rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/weight_min_f_contact_diff") 
    weight_contact_cost = rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/contact_force")  # minimizing the contact force
    weight_q_ddot = rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/small_q_p_ddot")# minimizing joint accelerations
    weight_hip_height_jump = rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/big_hip_jump") 
    weight_tip_clearance = rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/big_foot_tip_clearance") 
    weight_com_height = rospy.get_param("horizon/horizon_solver/problem_settings/cost_weights/weight_com_height") 

# common costs

weight_contact_cost = weight_contact_cost / cost_scaling_factor
prb.createIntermediateCost("min_f_contact", weight_contact_cost * cs.sumsqr(f_contact[0:2]))

weight_q_ddot = weight_q_ddot / cost_scaling_factor
prb.createIntermediateCost("min_q_ddot", weight_q_ddot * cs.sumsqr(
    q_p_ddot[1:]))  # minimizing the joint accelerations ("responsiveness" of the trajectory)

weight_hip_height_jump = weight_hip_height_jump / cost_scaling_factor
prb.createIntermediateCost("max_hip_height_jump", weight_hip_height_jump * cs.sumsqr(1 / (hip_position[2] + epsi)),
                        nodes=range(n_takeoff, n_touchdown))

weight_tip_clearance = weight_tip_clearance / cost_scaling_factor
prb.createIntermediateCost("max_foot_tip_clearance", weight_tip_clearance * cs.sumsqr(1 / (foot_tip_position[2] + epsi)),
                        nodes=range(n_takeoff, n_touchdown))

weight_com_height = weight_com_height / cost_scaling_factor
prb.createIntermediateCost("max_com_height", weight_com_height * cs.sumsqr(1/ ( com[2] + epsi)), 
                        nodes=range(n_takeoff, n_touchdown))

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

if resample_traj and not is_refine_phase:

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

## post-processing original solution

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
init_solution_foot_tip_position_aux = np.tile(foot_tip_position_init,(1, n_int + 1)) # auxiliary matrix to compute position excursion
init_solution_hip_position_aux = np.tile(hip_position_initial,(1, n_int + 1))
solution_v_foot_tip = dfk_foot(q=solution["q_p"], qdot=solution["q_p_dot"])["ee_vel_linear"]  # foot velocity
solution_v_foot_hip = dfk_hip(q=solution["q_p"], qdot=solution["q_p_dot"])["ee_vel_linear"]  # foot velocity

other_stuff = None
if not is_refine_phase:

    other_stuff = {"tau":cnstr_opt["tau_limits"], "i_q":i_q, "dt_opt":slvr.getDt(),
                 "foot_tip_height": np.transpose(solution_foot_tip_position-init_solution_foot_tip_position_aux[2,:]), 
                 "hip_height": np.transpose(solution_hip_position-init_solution_foot_tip_position_aux[2,:]), 
                 "tip_velocity": np.transpose(np.transpose(solution_v_foot_tip)),
                 "hip_velocity": np.transpose(np.transpose(solution_v_foot_hip)),
                 "sol_time": solution_time,
                 "is_calibrated": is_calibrated, 
                 "weight_min_input_diff": weight_min_input_diff, 
                 "weight_min_f_contact_diff": weight_min_f_contact_diff, 
                 "weight_init_sol_tracking": weight_init_sol_tracking, 
                 "is_refine_phase": is_refine_phase, 
                 "unique_id": unique_id}

else:
    
    other_stuff = {"tau":cnstr_opt["tau_limits"], "i_q":i_q, "dt_opt":slvr.getDt(),
                 "foot_tip_height": np.transpose(solution_foot_tip_position-init_solution_foot_tip_position_aux[2,:]), 
                 "hip_height": np.transpose(solution_hip_position-init_solution_foot_tip_position_aux[2,:]), 
                 "tip_velocity": np.transpose(np.transpose(solution_v_foot_tip)),
                 "hip_velocity": np.transpose(np.transpose(solution_v_foot_hip)),
                 "sol_time": solution_time,
                 "is_calibrated": is_calibrated, 
                 "weight_contact_cost": weight_contact_cost, 
                 "weight_hip_height_jump": weight_hip_height_jump, 
                 "weight_tip_clearance": weight_tip_clearance, 
                 "weight_min_input_diff": weight_min_input_diff, 
                 "weight_min_f_contact_diff": weight_min_f_contact_diff, 
                 "weight_com_height": weight_com_height, 
                 "is_refine_phase": is_refine_phase, 
                 "unique_id": unique_id}


sol_dict_full = {}

if resample_traj and not is_refine_phase:

    ## post-processing original solution

    # Hip and knee quadrature current estimation
    n_res_samples = len(p_res[0, :])
    i_q_res = np.zeros((n_q - 1, n_res_samples - 1))

    for i in range(n_q - 1):
        compensated_tau = tau_res[i + 1, :] + K_d0[i] * np.tanh( tanh_coeff * v_res[i + 1, 1:(n_res_samples)]) +\
                                              K_d1[i] * v_res[i + 1, 1:(n_res_samples)]
        i_q_res[i, :] = (rotor_axial_MoI[i] * a_res[i + 1, 0:(n_res_samples)] / red_ratio[i] + \
                        compensated_tau * red_ratio[i] / efficiency[i]) / K_t[i]

    res_GRF = sol_contact_map_res["tip"]
    res_q_p = p_res
    res_foot_tip_position = fk_foot(q = p_res)["ee_pos"][2,:].toarray()  # foot position
    res_hip_position = fk_hip(q=p_res)["ee_pos"][2,:].toarray()   # hip position
    init_res_foot_tip_position_aux = np.tile(foot_tip_position_init,(1, n_res_samples )) # auxiliary matrix to compute position excursion
    init_res_hip_position_aux = np.tile(hip_position_initial,(1, n_res_samples ))
    res_v_foot_tip = dfk_foot(q=p_res, qdot=v_res)["ee_vel_linear"]  # foot velocity
    res_v_foot_hip = dfk_hip(q=p_res, qdot=v_res)["ee_vel_linear"]  # foot velocity
    dt_res_vector = np.tile(dt_res, n_res_samples - 1)

    useful_solutions_res={"q_p_res":p_res,"q_p_dot_res":v_res, "q_p_ddot_res":a_res,
                    "tau_res":tau_res, "f_contact_res":res_GRF, "i_q_res":i_q_res, "dt_opt_res":dt_res_vector,
                    "foot_tip_height_res":np.transpose(res_foot_tip_position-init_res_foot_tip_position_aux[2,:]), 
                    "hip_height_res":np.transpose(res_hip_position-init_res_foot_tip_position_aux[2,:]), 
                    "tip_velocity_res":np.transpose(np.transpose(res_v_foot_tip)),
                    "hip_velocity_res":np.transpose(np.transpose(res_v_foot_hip))}

    sol_dict_full = {**solution,
            **useful_solutions_res, 
            **cnstr_opt,
            **other_stuff}

    ms.store(sol_dict_full) # saving solution data to file

else:

    sol_dict_full = {**solution,
            **cnstr_opt,
            **other_stuff}

    ms.store(sol_dict_full) # saving solution data to file

## replaying traj on rviz
replay_traj = rospy.get_param("/horizon/replay_trajectory")

if replay_traj:
    joint_names = urdf_awesome_leg.joint_names()
    joint_names.remove("universe")  # removing the "universe joint"

    if resample_traj and not is_refine_phase:

        rpl_traj = replay_trajectory(dt_res, joint_names, p_res, sol_contact_map_res, \
                    cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED, urdf_awesome_leg)  # replaying the (resampled) trajectory
    
    else:

        rpl_traj = replay_trajectory(slvr.getDt()[0], joint_names, solution["q_p"])  # replaying the (resampled) trajectory

    rpl_traj.sleep(1.)
    rpl_traj.replay(is_floating_base = False)

