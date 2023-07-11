import yaml

import time

import casadi as cs
import casadi_kin_dyn
import numpy as np

from horizon.problem import Problem
from horizon.ros.replay_trajectory import *
from horizon.solvers import solver
from horizon.transcriptions import transcriptor
from horizon.utils import kin_dyn, mat_storer, resampler_trajectory, utils
from jump_utils.horizon_utils import compute_required_iq_estimate

class landingEnergyRecover:

    def __init__(self, yaml_path: str, actuators_yaml_path: str,
                 urdf_path: str,
                 results_path: str,
                 sol_mat_name="energy_recov",
                 yaml_tag="energy_recov_opt", 
                 is_ref_prb = False):

        self.is_ref_prb = is_ref_prb
        
        self.yaml_tag = yaml_tag

        self.yaml_path = yaml_path
        self.actuators_yaml_path = actuators_yaml_path
        self.urdf_path = urdf_path
        self.results_path = results_path

        self.__parse_config_yamls()

        self.sol_mat_name = sol_mat_name

        self.__init_sol_dumpers()

        if self.is_ref_prb:
            
            self.__load_ig_from_mat()

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

        yaml_info = self.yaml_file[self.yaml_tag]

        # general parameters

        self.slvr_opt = {"ipopt.tol": yaml_info["solver"]["ipopt_tol"],
                        "ipopt.max_iter": yaml_info["solver"]["ipopt_maxiter"],
                        "ipopt.constr_viol_tol": yaml_info["solver"]["ipopt_cnstr_viol_tol"],
                        "ipopt.linear_solver": yaml_info["solver"]["ipopt_lin_solver"]
                        }
        
        self.slvr_name = yaml_info["solver"]["name"]

        self.trans_name = yaml_info["transcription"]["name"]
        self.trans_integrator = yaml_info["transcription"]["integrator_name"]

        self.jnt_limit_margin = abs(self.yaml_file[self.yaml_tag]["problem"]["jnt_limit_margin"])
        self.jnt_vel_limit_margin = abs(self.yaml_file[self.yaml_tag]["problem"]["jnt_vel_limit_margin"])

        self.tanh_coeff = yaml_info["i_q_estimation"]["tanh_coeff"]

        self.braking_q_dot =  yaml_info["problem"]["max_braking_q_dot"]

        self.landing_vel = yaml_info["problem"]["landing_vel"]

        self.n_actuators = len(self.act_yaml_file["K_d0"])
        self.is_iq_cnstrnt = self.yaml_file[self.yaml_tag]["problem"]["is_iq_cnstrnt"]
        self.is_friction_cone = self.yaml_file[self.yaml_tag]["problem"]["is_friction_cone"]
        self.mu_friction_cone = abs(self.yaml_file[self.yaml_tag]["problem"]["friction_cnstrnt"]["mu_friction_cone"])

        self.use_braking_constraint = yaml_info["problem"]["use_braking_constraint"]

        self.dt_lb = yaml_info["problem"]["dt_lb"]
        self.dt_ub = yaml_info["problem"]["dt_ub"]

        self.R_q = np.array(self.act_yaml_file["R"])
        self.L_m = self.act_yaml_file["L_m"]
        self.K_t = self.act_yaml_file["K_t"]
        self.red_ratio = np.array(self.act_yaml_file['red_ratio'])

        self.q_ig = np.array(self.yaml_file[self.yaml_tag]["problem"]["q_ig"])

        if not self.is_ref_prb:

            # ig generation

            self.yaml_tag_prb = self.yaml_file[self.yaml_tag]["problem"]["ig_generation"]

            self.is_q_ig = self.yaml_tag_prb["is_q_ig"]
             
            self.n_int = self.yaml_tag_prb['n_int']

            self.yaml_tag_ig = self.yaml_file[self.yaml_tag]["problem"]["ref"]
            
            self.n_int_ig = self.yaml_tag_ig['n_int']
        
        else:

            # actual solution generation

            self.yaml_tag_prb = self.yaml_file[self.yaml_tag]["problem"]["ref"]
            
            self.n_int = self.yaml_tag_prb['n_int']

        yaml_info_weights = self.yaml_tag_prb["weights"]

        self.weight_f_contact_cost = yaml_info_weights["weight_f_contact"]
        self.weight_f_contact_diff = yaml_info_weights["weight_f_contact_diff"]
        self.weight_jnt_input = yaml_info_weights['weight_jnt_input']
        self.weight_jnt_input_diff = yaml_info_weights['weight_jnt_input_diff']
        self.weight_q_dot = yaml_info_weights["weight_q_dot"]
        self.weight_q_dot_diff = yaml_info_weights["weight_q_dot_diff"]

        self.weight_reg_energy = yaml_info_weights["weight_reg_energy"]
        self.weight_impact_min = yaml_info_weights["weight_impact_min"]

        # self.weight_braking_ig = yaml_info_weights["weight_braking"]
        # self.weight_reg_q_dot_ig = yaml_info_weights['weight_reg_q_dot']

        self.weight_q_reg = yaml_info_weights["weight_q_reg"]

        # self.weight_imp_cntrl_ig = yaml_info_weights["weight_imp_cntrl"]
                
        self.n_nodes = self.n_int + 1
        self.last_node = self.n_nodes - 1
        self.input_nodes= list(range(0, self.n_nodes - 1))
        self.input_diff_nodes = list(range(1, self.n_nodes - 1))
        self.reg_pow_node_offset = self.n_int - (round(yaml_info["problem"]["reg_pow_node_offset"] * (self.n_int + 1) ) - 1)
        self.reg_pow_nodes = list(range(self.reg_pow_node_offset, self.n_nodes - 1))
    
    def __apply_loaded_ig(self):

        self.q_landing.setInitialGuess(self.q_landing_ig)

        self.q_p.setInitialGuess(self.q_p_ig)

        self.q_p_dot.setInitialGuess(self.q_p_dot_ig)

        self.q_p_ddot.setInitialGuess(self.q_p_ddot_ig)

        self.f_contact.setInitialGuess(self.f_contact_ig)

    def __load_ig_from_mat(self):

        self.__init_loader()

        loaded_sol_ig = self.ig_loader.load()
        loaded_sol_ig_res = self.ig_res_loader.load()

        self.T_landing = loaded_sol_ig["dt_opt"][0][0] *  loaded_sol_ig["n_int"][0][0]
        self.q_landing_ig = loaded_sol_ig["q_landing"]

        self.q_p_ig = loaded_sol_ig_res["q_p"]
        self.q_p_dot_ig = loaded_sol_ig_res["q_p_dot"]
        self.q_p_ddot_ig = loaded_sol_ig_res["q_p_ddot"]
        self.f_contact_ig = loaded_sol_ig_res["f_contact"]

        self.dt_ref = self.T_landing / self.n_int

    def __init_loader(self):

        self.ig_loader = mat_storer.matStorer(self.results_path + "/" + self.sol_mat_name + "_ig.mat") 
        self.ig_res_loader = mat_storer.matStorer(self.results_path + "/" + self.sol_mat_name + "_ig_res.mat") 

    def __init_sol_dumpers(self):

        if not self.is_ref_prb:

            self.ms_sol = mat_storer.matStorer(self.results_path + "/" + self.sol_mat_name + "_ig.mat")  # original opt. sol

            self.ms_resampl = mat_storer.matStorer(self.results_path + "/" + self.sol_mat_name + "_ig_res.mat")  # resampled opt. sol
        
        else:

            self.ms_sol = mat_storer.matStorer(self.results_path + "/" + self.sol_mat_name + ".mat")  # original opt. sol

    def __init_prb_common(self):

        self.urdf = open(self.urdf_path, "r").read()
        self.urdf_kin_dyn = casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn(self.urdf)

        self.n_q = self.urdf_kin_dyn.nq()  # number of joints
        self.n_v = self.urdf_kin_dyn.nv()  # number of dofs

        # self.I_lim = self.act_yaml_file["I_peak"]  # i_q current limits

        jnt_lim_margin_array = np.tile(self.jnt_limit_margin, (self.n_q))
        v_bounds = np.array(self.act_yaml_file["omega_max_nl_af44"])
        v_bounds = v_bounds - v_bounds * self.jnt_vel_limit_margin

        self.lbs = self.urdf_kin_dyn.q_min() + jnt_lim_margin_array
        self.ubs = self.urdf_kin_dyn.q_max() - jnt_lim_margin_array

        self.tau_lim = np.array([0] + self.act_yaml_file["tau_peak_ar"])  # effort limits (0 on the passive d.o.f.)

        self.prb = Problem(self.n_int)  # initialization of a problem object

        # Creating the state variables
        self.q_p = self.prb.createStateVariable("q_p", self.n_q)
        self.q_p_dot = self.prb.createStateVariable("q_p_dot",
                                                    self.n_v)  # here q_p_dot is actually not the derivative of the lagrangian state vector
        self.q_p.setBounds(self.lbs, self.ubs)
        self.q_p_dot[1:3].setBounds(- v_bounds, v_bounds)

        # v0 = np.zeros(self.n_v)
        # self.q_p_dot[0].setBounds(-1, -1, nodes=0)

        # Defining the input/s (joint accelerations)
        self.q_p_ddot = self.prb.createInputVariable("q_p_ddot", self.n_v)
        self.xdot = utils.double_integrator(self.q_p, self.q_p_dot, self.q_p_ddot)  # building the full state

        # Creating an additional input variable for the contact forces on the foot tip
        self.f_contact = self.prb.createInputVariable("f_contact", 3)  # dimension 3, no contact torques

        self.kp = self.prb.createSingleVariable('kp', self.n_v - 1)  # joint impedance gains (only actuated joints)
        self.kd = self.prb.createSingleVariable('kd', self.n_v - 1)
        kp_min = 0. * np.ones(self.n_v - 1)
        kp_max = 4000 * np.ones(self.n_v - 1)
        kd_min = 0.0 * np.ones(self.n_v - 1)
        kd_max = 100 * np.ones(self.n_v - 1)
        self.kp.setBounds(kp_min, kp_max)
        self.kd.setBounds(kd_min, kd_max)

        self.v_ee_param = self.prb.createSingleParameter('v_ee_param', 1)  # vertical vel component of tip @ touchdown

        self.q_p_dot_pretouchdown = cs.veccat(self.v_ee_param, 0.0, 0.0)

        # self.impact_z = self.prb.createSingleVariable('impact', 1) # impact --> lim_{dt->0}{int_{0}^{dt}{f * d_tau}}
        # self.impact = cs.veccat(0.0, 0.0, self.impact_z, 0.0, 0.0, 0.0) # impact --> lim_{dt->0}{int_{0}^{dt}{f * d_tau}}
        self.impact = self.prb.createSingleVariable('impact', 6)  # impact --> lim_{dt->0}{int_{0}^{dt}{f * d_tau}}
        impact_bounds = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.impact.setBounds(impact_bounds, impact_bounds)
        self.impact[2].setBounds(-cs.inf, cs.inf)
        # (vertical impact for simplicity)

        self.q_landing = self.prb.createSingleVariable('q_landing', self.n_q - 1)
    
    def __init_prb(self):
        
        self.__init_prb_common() # initializes stuff which is common between the ig and the ref prbs

        if not self.is_ref_prb:
            
            if (self.is_q_ig):
            
                self.q_landing.setInitialGuess(self.q_ig)

            self.dt_single_var = self.prb.createSingleVariable("dt_opt", 1) 
            self.dt_single_var.setBounds(self.dt_lb, self.dt_ub)  # bounds on dt
            dt = [self.dt_single_var] * (self.n_int)  # holds the complete time list

            self.prb.setDt(dt)
        
        else:
            
            self.__apply_loaded_ig()

            self.prb.setDt(self.dt_ref)

    def __get_quantities_from_urdf(self):

        # hip link pos
        self.fk_hip = self.urdf_kin_dyn.fk("base_link")  # deserializing
        self.hip_position = self.fk_hip(q=self.q_p)["ee_pos"]  # hip position (symbolic)

        # self.urdf_kin_dyn.crba() // inertia matrix
        # hip vel
        self.dfk_hip = self.urdf_kin_dyn.frameVelocity("base_link", \
                                                       casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)
        self.v_hip = self.dfk_hip(q=self.q_p, qdot=self.q_p_dot)["ee_vel_linear"]  # foot velocity

        # knee link pos
        self.fk_knee = self.urdf_kin_dyn.fk("shin1")  # deserializing
        self.knee_position = self.fk_knee(q=self.q_p)["ee_pos"]  # hip position (symbolic)

        # foot tip pos
        self.fk_foot = self.urdf_kin_dyn.fk("tip1")
        self.foot_tip_position = self.fk_foot(q=self.q_p)["ee_pos"]  # foot position

        #  tip jacobian
        self.fk_foot = self.urdf_kin_dyn.jacobian("tip1",
                                                  casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)
        self.J_tip = self.fk_foot(q=self.q_p)['J']

        self.B = self.urdf_kin_dyn.crba()(q=self.q_p)['B']
        self.B_Delta_v = self.B @ (self.q_p_dot - self.q_p_dot_pretouchdown)  # we assume q_p_dot pre impact = 0

        self.lambda_inv = self.J_tip @ cs.inv(self.B) @ self.J_tip.T  # like using forward dynamics -> not good

        # foot tip vel
        self.dfk_foot = self.urdf_kin_dyn.frameVelocity("tip1", \
                                                        casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)
        self.v_foot_tip = self.dfk_foot(q=self.q_p, qdot=self.q_p_dot)["ee_vel_linear"]  # foot velocity

        # center of mass
        self.com_fk = self.urdf_kin_dyn.centerOfMass()
        self.com = self.com_fk(q=self.q_p, v=self.q_p_dot, a=self.q_p_ddot)["com"]
        self.vcom = self.com_fk(q=self.q_p, v=self.q_p_dot, a=self.q_p_ddot)["vcom"]

        # impact torques

        self.JT_i = self.J_tip.T @ self.impact

        self.chi_dot_pretouchdown = self.J_tip @ self.q_p_dot_pretouchdown

        self.J_delta_v = self.J_tip @ (self.q_p_dot - self.q_p_dot_pretouchdown)

        #  to add
        self.i_q_estimate = cs.veccat(
            *compute_required_iq_estimate(self.act_yaml_file, self.q_p_dot, self.q_p_ddot, self.tau, self.tanh_coeff))
        self.L_q = 3 / 2 * np.array(self.L_m)

        self.R_q_mat = np.zeros((2, 2))
        self.R_q_mat[0, 0] = self.R_q[0]
        self.R_q_mat[1, 1] = self.R_q[1]

        self.L_q_mat = np.zeros((2, 2))
        self.L_q_mat[0, 0] = self.L_q[0]
        self.L_q_mat[1, 1] = self.L_q[1]

        self.p_batt, self.r_iq_2, \
            self.l_iq_i, self.l_iq_f, self.t_w = \
                            self.__compute_p_batt_fun(self.q_p_dot, 
                                    self.q_p_ddot, 
                                    self.tau)
        
        # variation of kin energy due to impact (both formulations are math equivalent, but the first turned out to be
        # more stable numerically)

        self.delta_ek = 1/2 * self.impact.T @ (self.J_tip @ self.q_p_dot_pretouchdown) # this is Ek_pst_takeoff - Ek_pre_takeoff <= 0 always for
        # energy conservation 
        
        # self.delta_ek = 1 / 2 * (
        #             self.q_p_dot.T @ self.B @ self.q_p_dot - self.q_p_dot_pretouchdown.T @ self.B @ self.q_p_dot_pretouchdown)  # this is Ek_pst_takeoff - Ek_pre_takeoff <= 0 always for
    
    def __set_constraints(self):

        self.tau_limits = self.prb.createIntermediateConstraint("tau_limits", self.tau / self.n_nodes)  # torque limits
        self.tau_limits.setBounds(- self.tau_lim / self.n_nodes, self.tau_lim / self.n_nodes)  # setting input limits

        self.prb.createConstraint("foot_vel_zero", self.v_foot_tip / self.n_nodes)  # we always have contact
        self.prb.createConstraint("tip_starts_on_ground", self.foot_tip_position[2] / self.n_nodes, nodes=0)  # tip starts on ground

        self.imp_cntrl_cntrnt = []

        for i in range(0, self.n_v - 1):

            self.imp_cntrl_cntrnt.append(self.prb.createIntermediateConstraint(f'imp_cntrl_jnt{i}', 
                                                    (self.tau[i + 1] - (
                                                    - self.kp[i] * (self.q_p[i + 1] - self.q_landing[i]) 
                                                    - self.kd[i] * self.q_p_dot[i + 1])) / self.n_nodes
                                                    ))

            # if self.use_soft_imp_cntrl:
            #
            #     self.imp_cntrl_cntrnt[i].setBounds(-cs.inf,
            #                                        cs.inf)  # we relax the contraint but keep it in the dumped data for debugging
            #
            # else:
            #
            #     self.imp_cntrl_cntrnt[i].setBounds(-1., 1.)  # we allow for some error in the constraint

        self.prb.createConstraint('q_p_at_touchdown', (self.q_p[1:] - self.q_landing) / self.n_nodes, nodes=0)

        for i in range(0, self.n_v):
            self.prb.createConstraint('impact_dyn_jnt' + str(i), (self.B_Delta_v[i] - self.JT_i[i]) / self.n_nodes,
                                      nodes=0)  # impact law

        grf_positive = self.prb.createIntermediateConstraint("grf_positive", self.f_contact[2] / self.n_nodes)
        grf_positive.setBounds(0., cs.inf)

        no_energy_violation = self.prb.createConstraint("we_cannot_create_energy_at_touchdown", self.delta_ek  / self.n_nodes, nodes=0)
        no_energy_violation.setBounds(-cs.inf / self.n_nodes, 0.0)
        
        if (self.use_braking_constraint):

            braking_cnstrnt = self.prb.createConstraint("braking_q_dot", self.q_p_dot[0]  / self.n_nodes, nodes = self.last_node)
            braking_cnstrnt.setBounds(-self.braking_q_dot / self.n_nodes, self.braking_q_dot / self.n_nodes)

        hip_above_ground = self.prb.createConstraint("hip_above_ground", self.hip_position[2] / self.n_nodes)  # no ground penetration on all the horizon
        hip_above_ground.setBounds(0.0, cs.inf)

        knee_above_ground = self.prb.createConstraint("knee_above_ground", self.knee_position[2] / self.n_nodes)  # no ground penetration on all the horizon
        knee_above_ground.setBounds(0.0, cs.inf)

        ### Additional constraints just for debugging (to be removed on the final run)
        iq_check = self.prb.createIntermediateConstraint('iq_check', self.i_q_estimate)
        ub_iq = np.ones((2, 1)) * cs.inf
        lb_iq = - np.ones((2, 1)) * cs.inf
        iq_check.setBounds(lb_iq, ub_iq)
        # l_iq_i = self.prb.createIntermediateConstraint('p_induct_check0', self.l_iq_i)
        # l_iq_i.setBounds(-cs.inf, cs.inf)
        # l_iq_f = self.prb.createIntermediateConstraint('p_induct_checkf', self.l_iq_f)
        # l_iq_f.setBounds(-cs.inf, cs.inf)
        # p_joule_check = self.prb.createIntermediateConstraint('p_joule_check', self.r_iq_2)
        # p_joule_check.setBounds(-cs.inf, cs.inf)
        # t_w = self.prb.createIntermediateConstraint('p_mech_pow_check', self.t_w)
        # t_w.setBounds(-cs.inf, cs.inf)
        # p_batt_check = self.prb.createIntermediateConstraint('p_batt_check', self.p_batt)
        # p_batt_check.setBounds(-cs.inf, cs.inf)

        delta_ek_check = self.prb.createIntermediateConstraint('delta_ek_check', self.delta_ek, 
                                                nodes = 0)
        delta_ek_check.setBounds(-cs.inf, cs.inf)
        
        # regenerate_something_please = self.prb.createConstraint("regenerate_something",
        #                                                 self.__ebatt(len(self.reg_pow_nodes), self.last_node), 
        #                                                 nodes = self.last_node)  # no ground penetration on all the horizon
        # regenerate_something_please.setBounds(-cs.inf, cs.inf)

        # regenerate_everywhere = self.prb.createConstraint("regenerate_everywhere",
        #                                                 self.p_batt, 
        #                                                 nodes = self.reg_pow_nodes)  # no ground penetration on all the horizon
        # regenerate_everywhere.setBounds(1e-2, cs.inf)

    def __compute_p_batt_fun(self, 
                        q_p_dot, q_p_ddot, tau):

        i_q_estimate = cs.veccat(
            *compute_required_iq_estimate(self.act_yaml_file, q_p_dot, q_p_ddot, tau, self.tanh_coeff))

        r_iq_2 = -3 / 2 * i_q_estimate.T @ self.R_q_mat @ i_q_estimate
        l_iq_i = 3 / 4 * i_q_estimate.T @ self.L_q_mat @ i_q_estimate
        l_iq_f = -3 / 4 * i_q_estimate.T @ self.L_q_mat @ i_q_estimate
        t_w = - (self.K_t * i_q_estimate).T @ (q_p_dot[1:] / self.red_ratio)

        p_batt = t_w + r_iq_2 + l_iq_f + l_iq_i

        return p_batt, r_iq_2, l_iq_f, l_iq_i, t_w
    
    def __ebatt(self, 
            n_nodes, 
            start_node):
        
        if (self.last_node - start_node) < 0 or n_nodes > (self.n_nodes - (self.last_node - start_node)):

            raise Exception("One or both between n_nodes or start_node is not valid!")
        
        contact_map_offset = dict(tip1=self.f_contact.getVarOffset(- (self.last_node - start_node)))

        tau_offset = kin_dyn.InverseDynamics(self.urdf_kin_dyn, contact_map_offset.keys(), \
                                        casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED).call(
                        self.q_p.getVarOffset(- (self.last_node - start_node)), \
                        self.q_p_dot.getVarOffset(- (self.last_node - start_node)), 
                        self.q_p_ddot.getVarOffset(- (self.last_node - start_node)),
                        contact_map_offset)
        
        pbatt_int, _, _, _, _  = self.__compute_p_batt_fun(self.q_p_dot.getVarOffset(- (self.last_node - start_node)), 
                                self.q_p_ddot.getVarOffset(- (self.last_node - start_node)), 
                                tau_offset)
        
        for i in range(start_node - n_nodes + 1, start_node):
            
            offset = - ((self.last_node - start_node) + i + 1)

            contact_map_offset = dict(tip1=self.f_contact.getVarOffset(offset))

            tau_offset = kin_dyn.InverseDynamics(self.urdf_kin_dyn, contact_map_offset.keys(), \
                                           casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED).call(
                            self.q_p.getVarOffset(offset), \
                            self.q_p_dot.getVarOffset(offset), 
                            self.q_p_ddot.getVarOffset(offset),
                            contact_map_offset)
            
            pbatt_offset, _, _, _, _ = self.__compute_p_batt_fun(self.q_p_dot.getVarOffset(offset), 
                                            self.q_p_ddot.getVarOffset(offset), 
                                            tau_offset)
            
            pbatt_int = pbatt_int + pbatt_offset
        
        if not self.is_ref_prb:
            
            return pbatt_int * self.dt_single_var
        
        else:

            return pbatt_int * self.dt_ref

    def __set_costs(self):

        # if self.weight_reg_q_dot > 0:
        #     # self.prb.createCost("q_dot_reg_0", self.weight_reg_q_dot * (1 / (self.q_p_dot[0]**2 + 0.001)), nodes=0)
        #     # self.prb.createCost("q_dot_reg_1", self.weight_reg_q_dot * (1 / (self.q_p_dot[1]**2 + 0.001)), nodes=0)
        #     # self.prb.createCost("q_dot_reg_2", self.weight_reg_q_dot * (1 / (self.q_p_dot[2]**2 + 0.001)), nodes=0)
        #     self.prb.createCost("reg_start_q_dot", self.weight_reg_q_dot * cs.sumsqr(self.q_p_dot), nodes=0)

        # if self.weight_braking > 0:
        #     self.prb.createCost("break_at_the_of_the_horizon", self.weight_braking * cs.sumsqr(self.q_p_dot), nodes=self.last_node)

        if self.weight_q_reg > 0:
            costname = "stay_far_from_singular"
            print("ADDING COST: " + costname)
            self.prb.createCost(costname, 
                            self.weight_q_reg * cs.sumsqr(self.q_landing[1:] - self.q_ig))

        if self.weight_impact_min > 0:
            costname = "min_impact_residual_kin_energy_dissipation"
            print("ADDING COST: " + costname)
            self.prb.createCost(costname, - self.weight_impact_min * self.delta_ek,
                                nodes=[0])  # delta_ek is always <= 0

        if self.weight_reg_energy > 0:
            costname = "max_reg_energy"
            print("ADDING COST: " + costname)

            # self.prb.createIntermediateCost("reg_energy_joule", - self.weight_reg_energy * self.r_iq_2)
            # self.prb.createCost("reg_energy_indct_init", - self.weight_reg_energy * self.l_iq_i, nodes=[0])
            # self.prb.createCost("reg_energy_indct_fin", - self.weight_reg_energy * self.l_iq_f, nodes=self.n_nodes - 1)
            # self.prb.createIntermediateCost("reg_energy_mech", 1 / (self.weight_reg_energy  * self.t_w)
            
            # self.prb.createIntermediateCost("reg_pow_batt", 1 / (self.p_batt + 0.0001))
      
            self.prb.createIntermediateCost("reg_pow_batt",  self.weight_reg_energy / self.n_nodes * (1e2 -  self.p_batt), 
                                        nodes = self.reg_pow_nodes)
            
            # self.prb.createIntermediateCost(costname, self.weight_reg_energy * (1e2 - self.__ebatt(len(self.reg_pow_nodes))), 
            #                             nodes = self.last_node)

        # regularizations
        if self.weight_f_contact_cost > 0:

            costname = "min_f_contact"
            print("ADDING COST: " + costname)

            self.prb.createIntermediateCost(costname, \
                                            self.weight_f_contact_cost / self.n_nodes * cs.sumsqr(self.f_contact[0:2]),
                                            nodes=self.input_nodes)

        if self.weight_f_contact_diff > 0:

            costname = "min_f_contact_diff"
            print("ADDING COST: " + costname)
            
            self.prb.createIntermediateCost(costname, \
                                            self.weight_f_contact_diff / self.n_nodes * cs.sumsqr(
                                                self.f_contact - self.f_contact.getVarOffset(-1)),
                                            nodes=self.input_diff_nodes)

        if self.weight_q_dot > 0:

            costname = "min_q_dot"
            print("ADDING COST: " + costname)

            self.prb.createCost(costname, self.weight_q_dot / self.n_nodes * cs.sumsqr(self.q_p_dot),
                                nodes=range(1, self.last_node))

        if self.weight_q_dot_diff > 0:

            costname = "min_q_dot_diff"
            print("ADDING COST: " + costname)

            self.prb.createIntermediateCost(costname, self.weight_q_dot_diff / self.n_nodes * cs.sumsqr(
                self.q_p_dot - self.q_p_dot.getVarOffset(-1)))

        if self.weight_jnt_input > 0:

            costname = "min_q_ddot"
            print("ADDING COST: " + costname)

            self.prb.createIntermediateCost(costname, self.weight_jnt_input / self.n_nodes * cs.sumsqr(self.q_p_ddot[1:]),
                                            nodes=self.input_nodes)

        if self.weight_jnt_input_diff > 0:
            
            costname = "min_jnt_input_diff"
            print("ADDING COST: " + costname)

            self.prb.createIntermediateCost(costname, \
                                            self.weight_jnt_input_diff / self.n_nodes * (cs.sumsqr(self.q_p_ddot - self.q_p_ddot.getVarOffset(-1))), 
                                            nodes=self.input_diff_nodes)

        # if self.weight_imp_cntrl > 0 and self.use_soft_imp_cntrl:

        #     for i in range(0, self.n_v - 1):
        #         self.prb.createIntermediateCost(f'imp_cntrl_jnt{i}', (self.tau[i + 1] - (
        #                 self.kp[i] * (self.q_p[i + 1] - self.q_landing[i]) - self.kd[i] * self.q_p_dot[i + 1])) ** 2)

    def __get_solution(self):

        self.solution = self.slvr.getSolutionDict()  # extracting solution
        self.cnstr_opt = self.slvr.getConstraintSolutionDict()
        self.lambda_cnstrnt = self.slvr.getCnstrLmbdSolDict()
        self.tau_sol = self.cnstr_opt["tau_limits"]

    def __compute_postproc_sols(self):

        # self.i_q_estimate_sol = compute_required_iq_estimate_num(self.act_yaml_file, 
        #                                     self.solution['q_p_dot'], 
        #                                     self.solution['q_p_ddot'], 
        #                                     self.tau_sol, 
        #                                     self.tanh_coeff) # DO NOT USE!!! TO BE FIXED
        
        self.i_q_estimate_sol = self.cnstr_opt['iq_check'] # we use directly the value from the dummy constraint

        self.r_iq_2_sol = np.zeros([1, self.i_q_estimate_sol.shape[1]])
        self.t_w_sol = np.zeros([1, self.i_q_estimate_sol.shape[1]])

        self.l_iq_i_sol = np.zeros([1, 1])
        self.l_iq_f_sol = np.zeros([1, 1])

        # exit()
        for i in range(0, self.i_q_estimate_sol.shape[1]):
            self.r_iq_2_sol[:, i] = -3 / 2 * self.i_q_estimate_sol[:, i].T @ self.R_q_mat @ self.i_q_estimate_sol[:, i]
            self.t_w_sol[:, i] = - (self.K_t * self.i_q_estimate_sol[:, i]).T @ (self.solution['q_p_dot'][1:, i] / self.red_ratio)

        if self.is_ref_prb:

            self.solution['dt_opt'] = self.dt_ref

        self.l_iq_i_sol = 3 / 4 * self.i_q_estimate_sol[:, 0].T @ self.L_q_mat @ self.i_q_estimate_sol[:, 0] * self.solution['dt_opt']
        self.l_iq_f_sol = -3 / 4 * self.i_q_estimate_sol[:, -1].T @ self.L_q_mat @ self.i_q_estimate_sol[:, -1] * self.solution['dt_opt']

        self.p_batt = self.r_iq_2_sol + self.t_w_sol + self.l_iq_i_sol + self.l_iq_f_sol

        self.r_iq_2_sol_int = np.sum(self.r_iq_2_sol) * self.solution['dt_opt']
        self.t_w_sol_int = np.sum(self.t_w_sol) * self.solution['dt_opt']
        
        self.e_batt = self.r_iq_2_sol_int + self.l_iq_i_sol + self.l_iq_f_sol + self.t_w_sol_int

    def __postproc_sol(self):
        
        self.__compute_postproc_sols()
        
        if self.is_ref_prb:

            self.solution['dt_opt'] = self.dt_ref

        self.sol_dict = {**self.solution,
                        **self.cnstr_opt,
                        **self.lambda_cnstrnt,
                        **{'n_int': self.n_int},
                        **{'reg_pow_nodes': self.reg_pow_nodes},
                        **{'tau_sol': self.tau_sol},
                        **{'r_iq_2_sol': self.r_iq_2_sol},
                        **{'l_iq_i_sol': self.l_iq_i_sol},
                        **{'l_iq_f_sol': self.l_iq_f_sol},
                        **{'t_w_sol': self.t_w_sol},
                        **{'r_iq_2_sol_int': self.r_iq_2_sol_int},
                        **{'t_w_sol_int': self.t_w_sol_int},
                        **{'e_batt': self.e_batt}, 
                        **{'p_batt': self.p_batt}
                        }

    def __resample_sol(self):

        q_sym = cs.SX.sym('q', self.n_q)
        q_dot_sym = cs.SX.sym('q_dot', self.n_v)
        q_ddot_sym = cs.SX.sym('q_ddot', self.n_v)
        x = cs.vertcat(q_sym, q_dot_sym)
        x_dot = utils.double_integrator(q_sym, q_dot_sym, q_ddot_sym)

        # sol_contact_map = dict(tip1 = self.solution["f_contact"])  # creating a contact map for applying the input to the foot

        dt_res = (self.solution['dt_opt'][0][0] * self.n_int) / self.n_int_ig

        x_res = resampler_trajectory.resampler(self.solution["x_opt"], self.solution["u_opt"], \
                                                self.slvr.getDt().flatten(), 
                                                dt_res, 
                                                None, \
                                                self.prb.getIntegrator())
                                                                                            
        self.p_res = x_res[:self.n_q]
        self.v_res = x_res[self.n_q:]

        self.res_f_contact = resampler_trajectory.resample_input(self.solution["f_contact"],\
                                                self.slvr.getDt().flatten(), dt_res)
        self.res_f_contact_map = dict(tip1 = self.res_f_contact)
        
        self.a_res = resampler_trajectory.resample_input(self.solution["q_p_ddot"],\
                                        self.slvr.getDt().flatten(), dt_res)

        from jump_utils.horizon_utils import inv_dyn_from_sol
        self.tau_res = inv_dyn_from_sol(self.urdf_kin_dyn, 
                            self.p_res, self.v_res, self.a_res,\
                            casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED,\
                            self.res_f_contact_map)
            
    def __postproc_res_sol(self):

        self.__resample_sol()

        self.useful_res_data={"q_p":self.p_res,"q_p_dot":self.v_res, "q_p_ddot":self.a_res,
                        "tau":self.tau_res, 
                        "f_contact":self.res_f_contact}

    def __dump_sol2file(self):
        
        if not self.is_ref_prb:
            
            self.ms_sol.store(self.sol_dict)  # saving solution data to file
            self.ms_resampl.store(self.useful_res_data)

        else:

            self.ms_sol.store(self.sol_dict)  # saving solution data to file

    def init_prb(self):

        self.__init_prb()

    def setup_prb(self):

        self.contact_map = dict(tip1=self.f_contact)

        self.prb.setDynamics(self.xdot)  # setting the dynamics we are interested of in the problem object (xdot)

        transcriptor.Transcriptor.make_method(self.trans_name, self.prb,
                                              dict(integrator=self.trans_integrator))  # setting the transcriptor

        self.tau = kin_dyn.InverseDynamics(self.urdf_kin_dyn, self.contact_map.keys(), \
                                           casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED).call(
            self.q_p, 
            self.q_p_dot, self.q_p_ddot, 
            self.contact_map)

        self.__get_quantities_from_urdf()

        self.__set_constraints()

        self.__set_costs()

        self.v_ee_param.assign(self.landing_vel)

        return True

    def solve_prb(self):

        self.slvr = solver.Solver.make_solver(self.slvr_name, self.prb, self.slvr_opt)
        t = time.time()
        self.slvr.solve()  # solving
        self.solution_time = time.time() - t
        print(f'solved in {self.solution_time} s')

    def postproc_sol(self):

        self.__get_solution()

        if not self.is_ref_prb:

            self.__postproc_sol()

            self.__postproc_res_sol() # we also dump resampled solution

        else:
            
            self.__postproc_sol()

        self.__dump_sol2file()
