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


class landingEnergyRecover:

    def __init__(self, yaml_path: str, actuators_yaml_path: str,
                 urdf_path: str,
                 results_path: str,
                 sol_mat_name="energy_recov",
                 yaml_tag="energy_recov_opt"):

        self.yaml_tag = yaml_tag

        self.yaml_path = yaml_path
        self.actuators_yaml_path = actuators_yaml_path
        self.urdf_path = urdf_path
        self.results_path = results_path

        self.__parse_config_yamls()

        self.sol_mat_name = sol_mat_name

        self.__init_sol_dumpers()

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

        self.n_int = yaml_info['problem']['n_int']

        self.slvr_opt = {"ipopt.tol": yaml_info["solver"]["ipopt_tol"],
                         "ipopt.max_iter": yaml_info["solver"]["ipopt_maxiter"],
                         "ipopt.constr_viol_tol": yaml_info["solver"]["ipopt_cnstr_viol_tol"],
                         }

        yaml_info_weights = yaml_info["problem"]["weights"]

        self.landing_vel = yaml_info["problem"]["landing_vel"]
        
        self.slvr_name = yaml_info["solver"]["name"]

        self.trans_name = yaml_info["transcription"]["name"]
        self.trans_integrator = yaml_info["transcription"]["integrator_name"]

        self.weight_f_contact_cost = yaml_info_weights["weight_f_contact"]
        self.weight_f_contact_diff = yaml_info_weights["weight_f_contact_diff"]
        self.weight_jnt_input = yaml_info_weights['weight_jnt_input']
        self.weight_jnt_input_diff = yaml_info_weights['weight_jnt_input_diff']
        self.weight_q_dot = yaml_info_weights["weight_q_dot"]
        self.weight_q_dot_diff  = yaml_info_weights["weight_q_dot_diff"]

        self.weight_reg_energy  = yaml_info_weights["weight_reg_energy"]
        self.weight_impact_min  = yaml_info_weights["weight_impact_min"]

        self.n_actuators = len(self.act_yaml_file["K_d0"])
        self.is_iq_cnstrnt = self.yaml_file[self.yaml_tag]["problem"]["is_iq_cnstrnt"]
        self.is_friction_cone = self.yaml_file[self.yaml_tag]["problem"]["is_friction_cone"]
        self.mu_friction_cone = abs(self.yaml_file[self.yaml_tag]["problem"]["friction_cnstrnt"]["mu_friction_cone"])

        self.dt_lb = yaml_info["problem"]["dt_lb"]
        self.dt_ub = yaml_info["problem"]["dt_ub"]

        self.n_nodes = self.n_int + 1
        self.last_node = self.n_nodes - 1
        self.input_nodes = list(range(0, self.n_nodes - 1))
        self.input_diff_nodes = list(range(1, self.n_nodes - 1))

    def __init_sol_dumpers(self):

        self.ms_sol = mat_storer.matStorer(self.results_path + "/" + self.sol_mat_name + ".mat")  # original opt. sol

    def __init_prb(self, n_passive_joints=1):

        self.urdf = open(self.urdf_path, "r").read()
        self.urdf_kin_dyn = casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn(self.urdf)

        self.n_q = self.urdf_kin_dyn.nq()  # number of joints
        self.n_v = self.urdf_kin_dyn.nv()  # number of dofs

        # self.I_lim = self.act_yaml_file["I_peak"]  # i_q current limits

        v_bounds = np.array(self.act_yaml_file["omega_max_nl_af44"])

        self.lbs = self.urdf_kin_dyn.q_min()
        self.ubs = self.urdf_kin_dyn.q_max()

        self.tau_lim = np.array([0] + self.act_yaml_file["tau_peak_ar"])  # effort limits (0 on the passive d.o.f.)

        self.prb = Problem(self.n_int)  # initialization of a problem object

        dt_single_var = self.prb.createSingleVariable("dt_landing", 1)  # dt before the takeoff
        dt_single_var.setBounds(self.dt_lb, self.dt_ub)  # bounds on dt
        dt = [dt_single_var] * (self.n_int)  # holds the complete time list
        #####PROBLEM HERE: why if I set a variable dt I find symbolic variables in the solution???
        # dt = 0.01

        self.prb.setDt(dt)

        # Creating the state variables
        self.q_p = self.prb.createStateVariable("q_p", self.n_q)
        self.q_p_dot = self.prb.createStateVariable("q_p_dot",
                                                    self.n_v)  # here q_p_dot is actually not the derivative of the lagrangian state vector
        # self.q_p.setBounds(self.lbs, self.ubs)
        self.q_p_dot[1:3].setBounds(- v_bounds, v_bounds)

        # v0 = np.zeros(self.n_v)
        # self.q_p_dot[0].setBounds(-1, -1, nodes=0)

        # Defining the input/s (joint accelerations)
        self.q_p_ddot = self.prb.createInputVariable("q_p_ddot", self.n_v)
        self.xdot = utils.double_integrator(self.q_p, self.q_p_dot, self.q_p_ddot)  # building the full state

        # Creating an additional input variable for the contact forces on the foot tip
        self.f_contact = self.prb.createInputVariable("f_contact", 3)  # dimension 3, no contact torques

        self.kp = self.prb.createSingleVariable('kp', self.n_v - 1) # joint impedance gains (only actuated joints)
        self.kd = self.prb.createSingleVariable('kd', self.n_v - 1)
        kp_min = 0 * np.ones(self.n_v - 1)
        kp_max = 4000 * np.ones(self.n_v - 1)
        kd_min = 0 * np.ones(self.n_v - 1)
        kd_max = 300 * np.ones(self.n_v - 1)
        self.kp.setBounds(kp_min, kp_max)
        self.kd.setBounds(kd_min, kd_max)

        self.q_landing = self.prb.createSingleVariable('q_landing', self.n_q - 1)
    
        self.v_ee_param = self.prb.createSingleParameter('v_ee_param', 1) # vertical vel component of tip @ touchdown

        self.q_p_dot_pretouchdown = cs.veccat(self.v_ee_param, 0.0, 0.0)

        self.chi_dot_pretouchdown = cs.veccat(0.0, 0.0, self.v_ee_param, 0.0, 0.0, 0.0)
        self.delta_chi_impact = cs.veccat(0.0, 0.0, 0.0, 0.0, 0.0, 0.0) - self.chi_dot_pretouchdown # total "task" twist vector [[linear], [angular]] @ touchdown

        # self.impact_z = self.prb.createSingleVariable('impact', 1) # impact --> lim_{dt->0}{int_{0}^{dt}{f * d_tau}}
        # self.impact = cs.veccat(0.0, 0.0, self.impact_z, 0.0, 0.0, 0.0) # impact --> lim_{dt->0}{int_{0}^{dt}{f * d_tau}}
        self.impact = self.prb.createSingleVariable('impact', 6) # impact --> lim_{dt->0}{int_{0}^{dt}{f * d_tau}}
        impact_bounds = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.impact.setBounds(impact_bounds, impact_bounds)
        self.impact[2].setBounds(-cs.inf, cs.inf)
        #(vertical impact for simplicity)

        return self.prb
    
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
        self.B_Delta_v = self.B @ (self.q_p_dot - self.q_p_dot_pretouchdown) # we assume q_p_dot pre impact = 0

        self.lambda_inv = self.J_tip @ cs.inv(self.B) @ self.J_tip.T # like using forward dynamics -> not good

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
        self.chi_dot_pretouchdown_simb = self.J_tip @ self.q_p_dot_pretouchdown

        self.J_delta_v = self.J_tip @ (self.q_p_dot - self.q_p_dot_pretouchdown)

        # variation of kin energy due to impact

        # self.delta_ek = 1/2 * self.impact.T @ (self.delta_chi_impact + 2 * self.J_tip @ self.q_p_dot_pretouchdown) # this is Ek_pst_takeoff - Ek_pre_takeoff <= 0 always for 
        # energy conservation 

        self.delta_ek = 1/2 * (self.q_p_dot.T @ self.B @ self.q_p_dot - self.q_p_dot_pretouchdown.T @ self.B @ self.q_p_dot_pretouchdown) # this is Ek_pst_takeoff - Ek_pre_takeoff <= 0 always for 

    def __set_constraints(self):

        self.tau_limits = self.prb.createIntermediateConstraint("tau_limits", self.tau)  # torque limits
        self.tau_limits.setBounds(- self.tau_lim, self.tau_lim)  # setting input limits

        self.prb.createConstraint("foot_vel_zero", self.v_foot_tip) # we always have contact
        self.prb.createConstraint("tip_starts_on_ground", self.foot_tip_position[2], nodes=0) # tip starts on ground

        # for i in range(0, self.n_v - 1):
        #     self.prb.createIntermediateConstraint(f'imp_cntrl_jnt{i}', self.tau[i + 1] - (
        #                 self.kp[i] * (self.q_p[i + 1] - self.q_landing[i]) - self.kd[i] * self.q_p_dot[i + 1]))

        self.prb.createConstraint('q_p_at_touchdown', self.q_p[1:] - self.q_landing, nodes=0)

        for i in range(0, self.n_v):
            self.prb.createConstraint('impact_dyn_jnt' + str(i), self.B_Delta_v[i] - self.JT_i[i], nodes=0) # impact law

        # for i in range(0, 6):
        #     self.prb.createConstraint(f'post_impact_jnt_vel_{i}', self.delta_chi_impact[i] - self.J_delta_v[i], nodes=0)

        # self.prb.createConstraint(f'post_impact_jnt_vel0', self.delta_chi_impact[0] - self.J_delta_v[0], nodes=0)
        # self.prb.createConstraint(f'post_impact_jnt_vel1', self.delta_chi_impact[1] - self.J_delta_v[1], nodes=0)
        # self.prb.createConstraint(f'post_impact_jnt_vel2', self.delta_chi_impact[2] - self.J_delta_v[2], nodes=0)
        # self.prb.createConstraint(f'post_impact_jnt_vel3', self.delta_chi_impact[3] - self.J_delta_v[3], nodes=0)
        # self.prb.createConstraint(f'post_impact_jnt_vel4', self.delta_chi_impact[4] - self.J_delta_v[4], nodes=0)
        # self.prb.createConstraint(f'post_impact_jnt_vel5', self.delta_chi_impact[5] - self.J_delta_v[5], nodes=0)

        # self.prb.createConstraint(f'post_impact_jnt_vel0', self.chi_dot_pretouchdown[0] - self.chi_dot_pretouchdown_simb[1], nodes=0)
        # self.prb.createConstraint(f'post_impact_jnt_vel0', self.chi_dot_pretouchdown[1] - self.chi_dot_pretouchdown_simb[1], nodes=0)
        # self.prb.createConstraint(f'post_impact_jnt_vel1', self.chi_dot_pretouchdown[2] - self.chi_dot_pretouchdown_simb[2], nodes=0)
        # self.prb.createConstraint(f'post_impact_jnt_vel2', self.chi_dot_pretouchdown[3] - self.chi_dot_pretouchdown_simb[3], nodes=0)
        # self.prb.createConstraint(f'post_impact_jnt_vel0', self.chi_dot_pretouchdown[4] - self.chi_dot_pretouchdown_simb[0], nodes=0)
        # self.prb.createConstraint(f'post_impact_jnt_vel0', self.chi_dot_pretouchdown[5] - self.chi_dot_pretouchdown_simb[0], nodes=0)

        grf_positive = self.prb.createIntermediateConstraint("grf_positive", self.f_contact[2])
        grf_positive.setBounds(0.0001, cs.inf)
        
        # no_energy_violation = self.prb.createConstraint("we_cannot_create_energy_at_touchdown", self.delta_ek, nodes = 0)
        # no_energy_violation.setBounds(-cs.inf, 0.0)

        self.prb.createFinalConstraint("braking_q_dot", self.q_p_dot)
        self.prb.createConstraint("braking_q_ddot", self.q_p_ddot, nodes = self.last_node -1)

        l_cnr = self.prb.createConstraint('impact_ratio_check', (1 / self.lambda_inv[2, 2]))
        l_cnr.setBounds(-cs.inf, cs.inf)

        hip_above_ground = self.prb.createConstraint("hip_above_ground", self.hip_position[2])  # no ground penetration on all the horizon
        hip_above_ground.setBounds(0.0, cs.inf)

        knee_above_ground = self.prb.createConstraint("knee_above_ground", self.knee_position[2])  # no ground penetration on all the horizon
        knee_above_ground.setBounds(0.0, cs.inf)

    def __set_costs(self):

        if self.weight_impact_min > 0:

            self.prb.createCost("min_impact_residual_kin_energy_dissipation", - self.weight_impact_min * self.delta_ek, nodes = 0) # delta_ek is always <= 0

        # regularizations
        if self.weight_f_contact_cost > 0:
            self.prb.createIntermediateCost("min_f_contact", \
                self.weight_f_contact_cost * cs.sumsqr(self.f_contact[0:2]), nodes = self.input_nodes)

        if self.weight_f_contact_diff > 0:
            self.prb.createIntermediateCost("min_f_contact_diff",\
                self.weight_f_contact_diff * cs.sumsqr(self.f_contact - self.f_contact.getVarOffset(-1)), nodes = self.input_diff_nodes)
            
        if self.weight_q_dot > 0:
            self.prb.createCost("min_q_dot", self.weight_q_dot * cs.sumsqr(self.q_p_dot), nodes = range(1, self.last_node)) 

        if self.weight_q_dot_diff > 0:
            self.prb.createIntermediateCost("min_q_dot_diff", self.weight_q_dot_diff * cs.sumsqr(self.q_p_dot - self.q_p_dot.getVarOffset(-1)))

        if self.weight_jnt_input > 0:
            self.prb.createIntermediateCost("min_q_ddot", self.weight_jnt_input * cs.sumsqr(self.q_p_ddot[1:]), nodes = self.input_nodes) 

        if self.weight_jnt_input_diff > 0:
       
            jnt_input_diff = cs.sumsqr(self.q_p_ddot[1:] - self.q_p_ddot[1:].getVarOffset(-1))

            self.prb.createIntermediateCost("min_jnt_input_diff", \
                self.weight_jnt_input_diff * jnt_input_diff, nodes = self.input_diff_nodes)  
            
    def __get_solution(self):

        self.solution = self.slvr.getSolutionDict()  # extracting solution
        self.cnstr_opt = self.slvr.getConstraintSolutionDict()
        self.lambda_cnstrnt = self.slvr.getCnstrLmbdSolDict()
        self.tau_sol = self.cnstr_opt["tau_limits"]
    
    def __postproc_sol(self):

        self.sol_dict = {**self.solution,
                        **self.cnstr_opt,
                        **self.lambda_cnstrnt,
                        **{'n_int': self.n_int},
                        **{'tau_sol': self.tau_sol}}



        # **{'dt': self.prb.getDt()

    def __dump_sol2file(self):

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
            self.q_p, \
            self.q_p_dot, self.q_p_ddot, self.contact_map)

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

        self.__postproc_sol()

        self.__dump_sol2file()
