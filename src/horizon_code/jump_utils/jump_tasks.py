import yaml

import argparse

import time

import casadi as cs
from sympy import N
import casadi_kin_dyn
import numpy as np
import rospkg
from horizon.problem import Problem
from horizon.ros.replay_trajectory import *
from horizon.solvers import solver
from horizon.transcriptions import transcriptor
from horizon.utils import kin_dyn, mat_storer, resampler_trajectory, utils

from jump_utils.miscell_utils import str2bool
from jump_utils.horizon_utils import inv_dyn_from_sol

class ResJumpGen:

    def __init__(self, yaml_path: str, actuators_yaml_path: str, 
                urdf_path: str,
                sol_mat_name: str, sol_mat_name_res: str, 
                results_path: str,
                cost_epsi = 1.0):
        
        self.yaml_path = yaml_path
        self.actuators_yaml_path = actuators_yaml_path
        self.urdf_path = urdf_path
        self.results_path = results_path

        self.__parse_config_yamls()

        self.sol_mat_name = sol_mat_name
        self.res_sol_mat_name = sol_mat_name_res

        self.__init_sol_dumpers()

        self.cost_epsi = cost_epsi

    def __parse_config_yamls(self):

        self.yaml_file = None
        with open(self.yaml_path, "r") as stream:
            try:
                self.yaml_file = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        self.act_yaml_file = None
        with open(self.actuators_yaml_path, "r") as stream:
            try:
                self.act_yaml_file = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        self.__read_opts_from_yaml()

    def __read_opts_from_yaml(self):

        self.tanh_coeff = self.yaml_file["i_q_estimation"]["tanh_coeff"]

        self.n_int = self.yaml_file["problem"]["n_int"]
        self.n_takeoff = self.yaml_file["problem"]["n_takeoff"]
        self.n_touchdown = self.yaml_file["problem"]["n_touchdown"]

        self.dt_lb = self.yaml_file["problem"]["dt_lb"]
        self.dt_ub = self.yaml_file["problem"]["dt_ub"]

        self.jnt_limit_margin = self.yaml_file["problem"]["jnt_limit_margin"]
        self.jnt_vel_limit_margin = self.yaml_file["problem"]["jnt_vel_limit_margin"]

        self.slvr_opt = {"ipopt.tol": self.yaml_file["solver"]["ipopt_tol"],
            "ipopt.max_iter": self.yaml_file["solver"]["ipopt_maxiter"],
            "ipopt.constr_viol_tol": self.yaml_file["solver"]["ipopt_cnstr_viol_tol"],
            "ipopt.linear_solver": self.yaml_file["solver"]["ipopt_lin_solver"]
        }

        self.slvr_name = self.yaml_file["solver"]["name"] 

        self.trans_name = self.yaml_file["transcription"]["name"] 
        self.trans_integrator = self.yaml_file["transcription"]["integrator_name"] 

        self.n_actuators = len(self.act_yaml_file["K_d0"])

        self.is_iq_cnstrnt = self.yaml_file["problem"]["is_iq_cnstrnt"]
        
        self.is_friction_cone = self.yaml_file["problem"]["is_friction_cone"]

        self.mu_friction_cone = abs(self.yaml_file["problem"]["friction_cnstrnt"]["mu_friction_cone"])

        self.scale_factor_base = self.yaml_file["problem"]["weights"]["scale_factor_costs_base"]  

        self.weight_f_contact_diff = self.yaml_file["problem"]["weights"]["weight_f_contact_diff"]  
        self.weight_f_contact_cost = self.yaml_file["problem"]["weights"]["weight_f_contact"] 
        self.weight_q_dot = self.yaml_file["problem"]["weights"]["weight_q_p_dot"] 
        self.weight_q_ddot = self.yaml_file["problem"]["weights"]["weight_q_p_ddot"] 
        self.weight_q_p_ddot_diff = self.yaml_file["problem"]["weights"]["weight_q_p_ddot_diff"] 

        self.weight_com_height = self.yaml_file["problem"]["weights"]["weight_com_height"] 
        self.weight_hip_height = self.yaml_file["problem"]["weights"]["weight_hip_height"] 
        self.weight_tip_clearance = self.yaml_file["problem"]["weights"]["weight_tip_clearance"] 
        self.weight_tip_under_hip = self.yaml_file["problem"]["weights"]["weight_tip_under_hip"] 

        self.dt_res = self.yaml_file["resampling"]["dt"]

    def __init_sol_dumpers(self):

        self.ms_orig = mat_storer.matStorer(self.results_path + "/" + self.sol_mat_name + ".mat")  # original opt. sol
        self.ms_resampl = mat_storer.matStorer(self.results_path + "/" + self.res_sol_mat_name + ".mat")  # original opt. sol

    def __set_igs(self):
        
        self.q_p.setInitialGuess(np.array([0.0, 0.0, 0.0]))
        self.q_p_dot.setInitialGuess(np.array([0.0, 0.0, 0.0]))
        self.q_p_ddot.setInitialGuess(np.array([0.0, 0.0, 0.0]))

        self.f_contact[2].setInitialGuess(self.urdf_kin_dyn.mass() * 9.81)

    def __get_quantities_from_urdf(self):
        
        # hip link pos
        self.fk_hip = self.urdf_kin_dyn.fk("base_link")  # deserializing
        self.hip_position = self.fk_hip(q = self.q_p)["ee_pos"]  # hip position (symbolic)

        # hip vel
        self.dfk_hip = self.urdf_kin_dyn.frameVelocity("base_link",\
                    casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)
        self.v_hip = self.dfk_hip(q = self.q_p, qdot = self.q_p_dot)["ee_vel_linear"]  # foot velocity

        # knee link pos
        self.fk_knee = self.urdf_kin_dyn.fk("shin1")  # deserializing
        self.knee_position = self.fk_knee(q = self.q_p)["ee_pos"]  # hip position (symbolic)

        # foot tip pos
        self.fk_foot = self.urdf_kin_dyn.fk("tip1")
        self.foot_tip_position = self.fk_foot(q = self.q_p)["ee_pos"]  # foot position

        # foot tip vel
        self.dfk_foot = self.urdf_kin_dyn.frameVelocity("tip1",\
                casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)
        self.v_foot_tip = self.dfk_foot(q=self.q_p, qdot=self.q_p_dot)["ee_vel_linear"]  # foot velocity

        # center of mass
        self.com_fk = self.urdf_kin_dyn.centerOfMass()
        self.com = self.com_fk(q = self.q_p, v = self.q_p_dot, a = self.q_p_ddot)["com"]

    def __set_constraints(self):
        
        # constraints
        self.tau_limits = self.prb.createIntermediateConstraint("tau_limits", self.tau)  # torque limits
        self.tau_limits.setBounds(- self.tau_lim, self.tau_lim)  # setting input limits
        # self.tau_limits.setBounds(-np.array([0, cs.inf, cs.inf]), np.array([0, cs.inf, cs.inf]))  # setting input limits

        self.prb.createConstraint("foot_vel_bf_touchdown_zero", self.v_foot_tip,
                            nodes=self.pre_takeoff_nodes) 
        self.prb.createConstraint("foot_vel_aftr_touchdown_zero", self.v_foot_tip,
                            nodes=self.post_touchdown_nodes)

        self.prb.createConstraint("tip_starts_on_ground", self.foot_tip_position[2], nodes=0)  
        self.prb.createConstraint("tip_ends_on_ground", self.foot_tip_position[2], nodes=((self.n_int + 1) - 1)) 
    
        tip_above_ground = self.prb.createConstraint("tip_above_ground", self.foot_tip_position[2], nodes = self.flight_nodes)  # no ground penetration on all the horizoin
        tip_above_ground.setBounds(0.0, cs.inf)
        hip_above_ground = self.prb.createConstraint("hip_above_ground", self.hip_position[2])  # no ground penetration on all the horizoin
        hip_above_ground.setBounds(0.0, cs.inf)
        knee_above_ground = self.prb.createConstraint("knee_above_ground", self.knee_position[2])  # no ground penetration on all the horizoin
        knee_above_ground.setBounds(0.0, cs.inf)
        
        self.prb.createConstraint("GRF_zero", self.f_contact,
                            nodes=self.flight_nodes)  # 0 GRF during flight

        self.prb.createConstraint("init_joint_vel_zero", self.q_p_dot,
                            nodes=0) 
        self.prb.createFinalConstraint("final_joint_vel_zero", self.q_p_dot)

        # Keep the ESTIMATED (with the calibrated current model) quadrature currents within bounds

        if self.is_iq_cnstrnt: # highly non linear constraint due to approximate sign function--> makes optimization much worse
            # use it only in the refinement phase, since it is expensive
            self.compensated_tau = []
            self.i_q_cnstr = []
            for i in range(self.n_q -1):
                self.compensated_tau.append(self.tau[i + 1] + self.act_yaml_file["K_d0"][i] * np.tanh( self.tanh_coeff * self.q_p_dot[i + 1]) + self.act_yaml_file["K_d1"][i] * self.q_p_dot[i + 1])
                i_q = (self.act_yaml_file["rotor_axial_MoI"][i] * self.q_p_ddot[i + 1] / self.act_yaml_file["red_ratio"][i] +\
                    self.compensated_tau[i] * self.act_yaml_file["red_ratio"][i] / \
                    self.act_yaml_file["efficiency"][i]) / self.act_yaml_file["K_t"][i]
                self.i_q_cnstr.append( self.prb.createIntermediateConstraint("quadrature_current_joint" + str(i), i_q) )
                self.i_q_cnstr[i].setBounds(- self.I_lim[i], self.I_lim[i])  # setting input limits

        # friction constraints

        if self.is_friction_cone:

            # linearized friction cone (constraint need to be split in two because setBounds only takes numerical vals)
            friction_cone_bf_takeoff1 = self.prb.createConstraint("friction_cone_bf_takeoff1",\
                                                self.f_contact[1] - (self.mu_friction_cone * self.f_contact[2]),
                                nodes=self.pre_takeoff_nodes)
            friction_cone_bf_takeoff1.setBounds(-cs.inf, 0)
            friction_cone_bf_takeoff2 = self.prb.createConstraint("friction_cone_bf_takeoff2",\
                                                self.f_contact[1] + (self.mu_friction_cone * self.f_contact[2]),
                                nodes=self.pre_takeoff_nodes)
            friction_cone_bf_takeoff2.setBounds(0, cs.inf)

            friction_cone_aftr_touchdown1 = self.prb.createConstraint("friction_cone_aftr_touchdown1",\
                                                self.f_contact[1] - self.mu_friction_cone * self.f_contact[2],
                                nodes=self.post_touchdown_nodes[:-1]) # f_contact not active on last node
            friction_cone_aftr_touchdown1.setBounds(-cs.inf, 0)
            friction_cone_aftr_touchdown2 = self.prb.createConstraint("friction_cone_aftr_touchdown2",\
                                                self.f_contact[1] + self.mu_friction_cone * self.f_contact[2],
                                nodes=self.post_touchdown_nodes[:-1]) # f_contact not active on last node
            friction_cone_aftr_touchdown2.setBounds(0, cs.inf)

    def __scale_weights(self):

        cost_scaling_factor = self.dt_lb * self.n_int * self.scale_factor_base

        self.weight_f_contact_cost = self.weight_f_contact_cost / cost_scaling_factor

        self.weight_q_dot = self.weight_q_dot / cost_scaling_factor

        self.weight_q_ddot = self.weight_q_ddot / cost_scaling_factor

        self.weight_hip_height = self.weight_hip_height / cost_scaling_factor

        self.weight_tip_clearance = self.weight_tip_clearance / cost_scaling_factor

        self.weight_com_height = self.weight_com_height / cost_scaling_factor

        self.weight_q_p_ddot_diff = self.weight_q_p_ddot_diff / cost_scaling_factor

        self.weight_f_contact_diff = self.weight_f_contact_diff / cost_scaling_factor

        self.weight_tip_under_hip = self.weight_tip_under_hip / cost_scaling_factor

    def __set_costs(self):
        
        self.__scale_weights()

        self.prb.createIntermediateCost("min_f_contact", \
            self.weight_f_contact_cost * cs.sumsqr(self.f_contact[0:2]))

        self.prb.createIntermediateCost("min_q_dot", self.weight_q_dot * cs.sumsqr(
            self.q_p_dot[1:])) 

        self.prb.createIntermediateCost("min_q_ddot", self.weight_q_ddot * cs.sumsqr(
            self.q_p_ddot[1:])) 

        self.prb.createIntermediateCost("max_hip_height_jump", \
                                self.weight_hip_height * cs.sumsqr(1 / (self.hip_position + self.cost_epsi)),
                                nodes=self.flight_nodes)

        self.prb.createIntermediateCost("max_foot_tip_clearance", \
                                self.weight_tip_clearance * cs.sumsqr(1 / (self.foot_tip_position[2] + self.cost_epsi)),
                                nodes=self.flight_nodes)

        self.prb.createIntermediateCost("max_com_height", self.weight_com_height * cs.sumsqr(1/ ( self.com[2] + self.cost_epsi)), 
                                nodes=self.flight_nodes)

        self.prb.createIntermediateCost("min_input_diff", self.weight_q_p_ddot_diff * cs.sumsqr(self.q_p_ddot[1:] - self.q_p_ddot[1:].getVarOffset(-1)), 
                                    nodes = range(1, self.n_int))  

        self.prb.createIntermediateCost("min_f_contact_diff", self.weight_f_contact_diff * cs.sumsqr(self.f_contact - self.f_contact.getVarOffset(-1)), 
                                    nodes = range(1, self.n_int))

        self.prb.createIntermediateCost("max_tip_under_hip_first_node", 
                                    self.weight_tip_under_hip * (cs.sumsqr(self.hip_position[1] - self.foot_tip_position[1])), 
                                    nodes = 0)
        self.prb.createIntermediateCost("max_tip_under_hip_last_node", 
                                    self.weight_tip_under_hip * (cs.sumsqr(self.hip_position[1] - self.foot_tip_position[1])), 
                                    nodes = ((self.n_int + 1) - 1))

    def __get_solution(self):

        self.solution = self.slvr.getSolutionDict() # extracting solution
        self.cnstr_opt = self.slvr.getConstraintSolutionDict()
        self.lambda_cnstrnt = self.slvr.getCnstrLmbdSolDict()

        self.tau_sol = self.cnstr_opt["tau_limits"]

    def __resample_sol(self):

        q_sym = cs.SX.sym('q', self.n_q)
        q_dot_sym = cs.SX.sym('q_dot', self.n_v)
        q_ddot_sym = cs.SX.sym('q_ddot', self.n_v)
        x = cs.vertcat(q_sym, q_dot_sym)
        x_dot = utils.double_integrator(q_sym, q_dot_sym, q_ddot_sym)

        sol_contact_map = dict(tip1 = self.solution["f_contact"])  # creating a contact map for applying the input to the foot

        x_res = resampler_trajectory.resampler(self.solution["x_opt"], self.solution["u_opt"], \
                                                self.slvr.getDt().flatten(), self.dt_res, None, \
                                                self.prb.getIntegrator())
                                                                                            
        self.p_res = x_res[:self.n_q]
        self.v_res = x_res[self.n_q:]
        self.a_res = resampler_trajectory.resample_input(self.solution["q_p_ddot"],\
                                        self.slvr.getDt().flatten(), self.dt_res)

        self.res_f_contact = resampler_trajectory.resample_input(self.solution["f_contact"],\
                                                self.slvr.getDt().flatten(), self.dt_res)
        self.res_f_contact_map = dict(tip1 = self.res_f_contact)

        self.tau_res = inv_dyn_from_sol(self.urdf_kin_dyn, 
                                self.p_res, self.v_res, self.a_res,\
                                casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED,\
                                self.res_f_contact_map)

    def __postproc_raw_sol(self):
        
        # Hip and knee quadrature current estimation
        i_q_n_samples = len(self.solution["q_p_ddot"][0, :])
        i_q = np.zeros((self.n_q - 1, i_q_n_samples))
        for i in range(self.n_q - 1):
            compensated_tau = self.tau_sol[i + 1, :] + self.act_yaml_file["K_d0"][i] * np.tanh( self.tanh_coeff * self.solution["q_p_dot"][i + 1, 1:(i_q_n_samples + 1)]) + \
                                                self.act_yaml_file["K_d1"][i] * self.solution["q_p_dot"][i + 1, 1:(i_q_n_samples + 1)]
            i_q[i, :] = (self.act_yaml_file["rotor_axial_MoI"][i] * self.solution["q_p_ddot"][i + 1, :] / self.act_yaml_file["red_ratio"][i] +\
                        compensated_tau * self.act_yaml_file["red_ratio"][i] / self.act_yaml_file["efficiency"][i]) / self.act_yaml_file["K_t"][i]

        solution_q_p = self.solution["q_p"]
        solution_foot_tip_position = self.fk_foot(q = solution_q_p)["ee_pos"][2,:].toarray()  # foot position
        solution_hip_position = self.fk_hip(q=solution_q_p)["ee_pos"][2,:].toarray()   # hip position
        solution_v_foot_tip = self.dfk_foot(q=self.solution["q_p"], qdot=self.solution["q_p_dot"])["ee_vel_linear"]  # foot velocity
        solution_v_foot_hip = self.dfk_hip(q=self.solution["q_p"], qdot=self.solution["q_p_dot"])["ee_vel_linear"]  # foot velocity

        self.other_stuff = {"tau":self.cnstr_opt["tau_limits"],\
                    "i_q":i_q,\
                    "dt_opt":self.slvr.getDt(),
                    "foot_tip_height": np.transpose(solution_foot_tip_position), 
                    "hip_height": np.transpose(solution_hip_position), 
                    "tip_velocity": np.transpose(np.transpose(solution_v_foot_tip)),
                    "hip_velocity": np.transpose(np.transpose(solution_v_foot_hip)),
                    "sol_time": self.solution_time,
                    "weight_min_input_diff": self.weight_q_p_ddot_diff, 
                    "weight_min_f_contact_diff": self.weight_f_contact_diff, 
                    "is_refine_phase": False}

    def __postproc_res_sol(self):

        # Hip and knee quadrature current estimation
        n_res_samples = len(self.p_res[0, :])
        i_q_res = np.zeros((self.n_q - 1, n_res_samples - 1))

        for i in range(self.n_q - 1):
            compensated_tau = self.tau_res[i + 1, :] + self.act_yaml_file["K_d0"][i] * np.tanh( self.tanh_coeff * self.v_res[i + 1, 1:(n_res_samples)]) +\
                                                self.act_yaml_file["K_d1"][i] * self.v_res[i + 1, 1:(n_res_samples)]
            i_q_res[i, :] = (self.act_yaml_file["rotor_axial_MoI"][i] * self.a_res[i + 1, 0:(n_res_samples)] / self.act_yaml_file["red_ratio"][i] + \
                            compensated_tau * self.act_yaml_file["red_ratio"][i] / self.act_yaml_file["efficiency"][i]) / self.act_yaml_file["K_t"][i]

        res_foot_tip_position = self.fk_foot(q = self.p_res)["ee_pos"][2,:].toarray()  # foot position
        res_hip_position = self.fk_hip(q=self.p_res)["ee_pos"][2,:].toarray()   # hip position
        res_v_foot_tip = self.dfk_foot(q=self.p_res, qdot=self.v_res)["ee_vel_linear"]  # foot velocity
        res_v_foot_hip = self.dfk_hip(q=self.p_res, qdot=self.v_res)["ee_vel_linear"]  # foot velocity
        dt_res_vector = np.tile(self.dt_res, n_res_samples - 1)

        self.useful_solutions_res={"q_p":self.p_res,"q_p_dot":self.v_res, "q_p_ddot":self.a_res,
                        "tau":self.tau_res, "f_contact":self.res_f_contact, "i_q":i_q_res, "dt_opt":dt_res_vector,
                        "foot_tip_height":np.transpose(res_foot_tip_position), 
                        "hip_height":np.transpose(res_hip_position), 
                        "tip_velocity":np.transpose(np.transpose(res_v_foot_tip)),
                        "hip_velocity":np.transpose(np.transpose(res_v_foot_hip))}

    def __postproc_sol(self):

        self.__postproc_raw_sol()

        self.__postproc_res_sol()

        self.sol_dict_full_resampl= {**self.useful_solutions_res}

        self.sol_dict_full_orig = {**self.solution,
                **self.cnstr_opt,
                **self.lambda_cnstrnt,
                **self.other_stuff}

    def __dump_sol2file(self):

        self.ms_orig.store(self.sol_dict_full_orig) # saving solution data to file

        self.ms_resampl.store(self.sol_dict_full_resampl) # saving solution data to file

    def init_prb(self, n_passive_joints = 1):

        self.urdf = open(self.urdf_path, "r").read()
        self.urdf_kin_dyn = casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn(self.urdf)

        self.n_q = self.urdf_kin_dyn.nq()  # number of joints
        if not (self.n_q - n_passive_joints) == self.n_actuators:
            raise Exception("The number of actuators in " + self.actuators_yaml_path + " does not math the ones in the loaded URDF!!") 
        self.n_v = self.urdf_kin_dyn.nv()  # number of dofs

        self.I_lim = self.act_yaml_file["I_peak"] # i_q current limits

        jnt_lim_margin_array = np.tile(self.jnt_limit_margin, (self.n_q))
        v_bounds = np.array(self.act_yaml_file["omega_max_nl_af44"])
        v_bounds = v_bounds - v_bounds * self.jnt_vel_limit_margin

        self.lbs = self.urdf_kin_dyn.q_min() + jnt_lim_margin_array
        self.ubs = self.urdf_kin_dyn.q_max() - jnt_lim_margin_array

        self.tau_lim = np.array([0] + self.act_yaml_file["tau_peak_ar"])  # effort limits (0 on the passive d.o.f.)

        self.prb = Problem(self.n_int)  # initialization of a problem object

        dt1 = self.prb.createSingleVariable("dt1", 1)  # dt before the takeoff
        dt2 = self.prb.createSingleVariable("dt2", 1)  # dt during flight
        dt3 = self.prb.createSingleVariable("dt3", 1)  # dt after touchdown
        dt1.setBounds(self.dt_lb, self.dt_ub)  # bounds on dt1
        dt2.setBounds(self.dt_lb, self.dt_ub)  # bounds on dt2
        dt3.setBounds(self.dt_lb, self.dt_ub)  # bounds on dt3

        dt=[dt1]* (self.n_takeoff) + [dt2] * (self.n_touchdown - self.n_takeoff) +\
            [dt3] * (self.n_int - self.n_touchdown) # holds the complete time list

        self.prb.setDt(dt)

        # Creating the state variables
        self.q_p = self.prb.createStateVariable("q_p", self.n_q)
        self.q_p_dot = self.prb.createStateVariable("q_p_dot",
                                        self.n_v)  # here q_p_dot is actually not the derivative of the lagrangian state vector
        self.q_p.setBounds(self.lbs, self.ubs) 
        self.q_p_dot[1:3].setBounds(- v_bounds, v_bounds)

        # Defining the input/s (joint accelerations)
        self.q_p_ddot = self.prb.createInputVariable("q_p_ddot", self.n_v)  # using joint accelerations as an input variable

        self.xdot = utils.double_integrator(self.q_p, self.q_p_dot, self.q_p_ddot)  # building the full state

        # Creating an additional input variable for the contact forces on the foot tip
        self.f_contact = self.prb.createInputVariable("f_contact", 3)  # dimension 3
        
        return self.prb

    def setup_prb(self):
        
        self.__set_igs()

        self.f_contact[2].setLowerBounds(0)  # tip cannot be pulled from the ground

        self.contact_map = dict(tip1 = self.f_contact)  # creating a contact map for applying the input to the foot

        self.prb.setDynamics(self.xdot)  # setting the dynamics we are interested of in the problem object (xdot)

        transcriptor.Transcriptor.make_method(self.trans_name, self.prb, dict(integrator = self.trans_integrator))  # setting the transcriptor

        self.tau = kin_dyn.InverseDynamics(self.urdf_kin_dyn, self.contact_map.keys(), \
                casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED).call(self.q_p,\
                                                        self.q_p_dot, self.q_p_ddot, self.contact_map) 

        self.__get_quantities_from_urdf()

        self.pre_takeoff_nodes = range(0, self.n_takeoff + 1)
        self.post_touchdown_nodes = range(self.n_touchdown, self.n_int + 1)
        self.flight_nodes = range(self.n_takeoff + 1, self.n_touchdown)

        self.__set_constraints()

        self.__set_costs()

        return True 

    def solve_prb(self):

        self.slvr = solver.Solver.make_solver(self.slvr_name, self.prb, self.slvr_opt) 
        t = time.time()
        self.slvr.solve()  # solving
        self.solution_time = time.time() - t
        print(f'solved in {self.solution_time} s')

    def postproc_sol(self):

        self.__get_solution()
        
        self.__resample_sol()

        self.__postproc_sol()

        self.__dump_sol2file()


    