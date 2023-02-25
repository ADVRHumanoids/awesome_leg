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

        self.slvr_name = yaml_info["solver"]["name"]

        self.trans_name = yaml_info["transcription"]["name"]
        self.trans_integrator = yaml_info["transcription"]["integrator_name"]

        self.weight_jnt_input_diff = yaml_info_weights['weight_jnt_input_diff']
        self.weight_f_contact_diff = yaml_info_weights["weight_f_contact_diff"]
        self.weight_f_contact_cost = yaml_info_weights["weight_f_contact"]
        self.weight_q_dot = yaml_info_weights["weight_q_p_dot"]
        # self.weight_q_ddot         = yaml_info_weights["weight_q_p_ddot"]
        # self.weight_q_p_ddot_diff  = yaml_info_weights["weight_jnt_input_diff"]

        # self.weight_term_com_vel   = yaml_info_weights["weight_com_term_vel"]
        # self.weight_com_vel        = yaml_info_weights["weight_com_vel"]
        # self.weight_tip_under_hip  = yaml_info_weights["weight_tip_under_hip"]
        # self.weight_sat_i_q        = yaml_info_weights["weight_sat_i_q"]

        # self.weight_com_pos        = yaml_info_weights["weight_com_pos"]

        # self.n_actuators = len(self.act_yaml_file["K_d0"])
        # self.is_iq_cnstrnt = self.yaml_file[self.yaml_tag]["problem"]["is_iq_cnstrnt"]
        # self.sliding_guide_kd = self.yaml_file[self.yaml_tag]["sliding_guide_friction"]["kd"]
        # self.is_sliding_guide_friction = self.yaml_file[self.yaml_tag]["problem"]["is_sliding_guide_friction"]
        # self.is_friction_cone = self.yaml_file[self.yaml_tag]["problem"]["is_friction_cone"]
        # self.mu_friction_cone = abs(self.yaml_file[self.yaml_tag]["problem"]["friction_cnstrnt"]["mu_friction_cone"])

        self.dt_lb = yaml_info["problem"]["dt_lb"]
        self.dt_ub = yaml_info["problem"]["dt_ub"]

        self.n_nodes = self.n_int + 1
        self.last_node = self.n_nodes - 1
        self.input_nodes = list(range(0, self.n_nodes - 1))

    def __init_sol_dumpers(self):

        self.ms_sol = mat_storer.matStorer(self.results_path + "/" + self.sol_mat_name + ".mat")  # original opt. sol

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
        self.jac_foot_tip = self.urdf_kin_dyn.jacobian("tip1",
                                                       casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)

        self.b_q_p = self.urdf_kin_dyn.crba()(q=self.q_p)['B']


        # foot tip vel
        self.dfk_foot = self.urdf_kin_dyn.frameVelocity("tip1", \
                                                        casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)
        self.v_foot_tip = self.dfk_foot(q=self.q_p, qdot=self.q_p_dot)["ee_vel_linear"]  # foot velocity

        # center of mass
        self.com_fk = self.urdf_kin_dyn.centerOfMass()
        self.com = self.com_fk(q=self.q_p, v=self.q_p_dot, a=self.q_p_ddot)["com"]
        self.vcom = self.com_fk(q=self.q_p, v=self.q_p_dot, a=self.q_p_ddot)["vcom"]

    def __set_constraints(self):

        # constraints

        self.tau_limits = self.prb.createIntermediateConstraint("tau_limits", self.tau)  # torque limits
        self.tau_limits.setBounds(- self.tau_lim, self.tau_lim)  # setting input limits

        self.prb.createConstraint("foot_vel_zero", self.v_foot_tip)
        self.prb.createConstraint("tip_starts_on_ground", self.foot_tip_position[2], nodes=0)

        for dim_i in range(1, self.n_v):
            self.prb.createIntermediateConstraint(f'stiffness_damping_dim_{dim_i}', self.tau[dim_i] - (
                        self.kp[dim_i] * (self.q_p[dim_i] - self.q_init[dim_i]) - self.kd[dim_i] * self.q_p_dot[dim_i]))

        self.prb.createConstraint('q_p_impact_ref', self.q_p - self.q_init, nodes=0)

        self.prb.createConstraint('initial_vel', self.v_impact - (self.jac_foot_tip(q=self.q_init)['J'] @ self.q_p_dot), nodes=0)

        for dim_i in range(1, self.n_v):
            self.prb.createConstraint(f'impact_{dim_i}', (self.b_q_p @ self.q_p_dot)[dim_i, :] - self.impact[dim_i], nodes=0)


        # self.prb.createConstraint("tip_stays_on_ground", self.v_foot_tip, self.contact_nodes)

        # hip_above_ground = self.prb.createConstraint("hip_above_ground", self.hip_position[2])  # no ground penetration on all the horizon
        # hip_above_ground.setBounds(0.0, cs.inf)

        # tip_above_ground = self.prb.createConstraint("tip_above_ground", self.foot_tip_position[2])  # no ground penetration on all the horizon
        # tip_above_ground.setBounds(0.0, cs.inf)

        # com_towards_vertical = self.prb.createIntermediateConstraint("com_towards_vertical", self.vcom[2], self.contact_nodes) # intermediate, so all except last node
        # com_towards_vertical.setBounds(0.0, cs.inf)

        # self.prb.createIntermediateConstraint("grf_zero", self.f_contact, nodes = self.flight_nodes[:-1])  # 0 GRF during flight

        # grf_positive = self.prb.createIntermediateConstraint("grf_positive", self.f_contact[2], nodes = self.contact_nodes)  # 0 GRF during flight
        # grf_positive.setBounds(0.0, cs.inf)

        # self.prb.createConstraint("leg_starts_still", self.q_p_dot, nodes=0) # leg starts still

    def __set_costs(self):

        self.prb.createIntermediateCost("min_f_contact", \
                                        self.weight_f_contact_cost * cs.sumsqr(self.f_contact[0:2]),
                                        nodes=self.input_nodes)

        self.prb.createCost("min_q_dot", self.weight_q_dot * cs.sumsqr(self.q_p_dot))

        self.prb.createCost("min_kin", 1/2 * self.impact.T @ self.v_impact)




        # if self.weight_q_dot_diff > 0:
        #     self.prb.createIntermediateCost("min_q_dot_diff", self.weight_q_dot_diff * cs.sumsqr(self.q_p_dot - self.q_p_dot.getVarOffset(-1)))

        # if self.weight_jnt_input > 0:
        #     self.prb.createIntermediateCost("min_q_ddot", self.weight_jnt_input * cs.sumsqr(self.q_p_ddot[1:]), nodes = self.input_nodes)

        # if self.weight_com_vel > 0:
        #     self.prb.createIntermediateCost("max_com_vel", self.weight_com_vel * 1/ ( cs.sumsqr(self.vcom[2]) + 0.0001 ))

        # if self.weight_term_com_vel > 0:
        #     self.prb.createIntermediateCost("max_com_term_vel", self.weight_term_com_vel * 1/ ( cs.sumsqr(self.vcom[2]) + 0.0001 ), \
        #                                     nodes = self.contact_nodes[-1])

        #
        # if self.weight_jnt_input_diff > 0:
        #     jnt_input_diff = cs.sumsqr(self.tau[1:] - self.tau[1:].getVarOffset(-1))

        # self.prb.createIntermediateCost("min_jnt_input_diff", \
        #     self.weight_jnt_input_diff * jnt_input_diff, nodes = self.input_diff_nodes)

        # if self.weight_f_contact_diff > 0:
        #     self.prb.createIntermediateCost("min_f_contact_diff",\
        #         self.weight_f_contact_diff * cs.sumsqr(self.f_contact - self.f_contact.getVarOffset(-1)), nodes = self.input_diff_nodes)

        # if self.weight_tip_under_hip > 0:
        #     self.prb.createIntermediateCost("max_tip_under_hip", \
        #         self.weight_tip_under_hip * (cs.sumsqr(self.hip_position[1] - self.foot_tip_position[1])))

    def __get_solution(self):

        self.solution = self.slvr.getSolutionDict()  # extracting solution
        self.cnstr_opt = self.slvr.getConstraintSolutionDict()
        self.lambda_cnstrnt = self.slvr.getCnstrLmbdSolDict()

        self.tau_sol = self.cnstr_opt["tau_limits"]

    def __postproc_sol(self):

        self.sol_dict_full_raw_sol = {**self.solution,
                                      **self.cnstr_opt,
                                      **self.lambda_cnstrnt,
                                      **{'n_int': self.n_int},
                                      **{'tau_sol': self.tau_sol},
                                      **{'dt': self.prb.getDt()}}

    def __dump_sol2file(self):

        self.ms_sol.store(self.sol_dict_full_raw_sol)  # saving solution data to file

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

        # dt_sing_var = self.prb.createSingleVariable("dt", 1)  # dt before the takeoff
        # dt_sing_var.setBounds(self.dt_lb, self.dt_ub)  # bounds on dt3

        dt = 0.01
        # dt = [dt_sing_var] * (self.n_int)  # holds the complete time list

        self.prb.setDt(dt)

        # Creating the state variables
        self.q_p = self.prb.createStateVariable("q_p", self.n_q)
        self.q_p_dot = self.prb.createStateVariable("q_p_dot",
                                                    self.n_v)  # here q_p_dot is actually not the derivative of the lagrangian state vector
        # self.q_p.setBounds(self.lbs, self.ubs)
        self.q_p_dot[1:3].setBounds(- v_bounds, v_bounds)

        v0 = np.zeros(self.n_v)
        self.q_p_dot.setBounds(v0, v0, nodes=0)

        # Defining the input/s (joint accelerations)
        self.q_p_ddot = self.prb.createInputVariable("q_p_ddot", self.n_v)
        self.xdot = utils.double_integrator(self.q_p, self.q_p_dot, self.q_p_ddot)  # building the full state

        # Creating an additional input variable for the contact forces on the foot tip
        self.f_contact = self.prb.createInputVariable("f_contact", 3)  # dimension 3

        self.kp = self.prb.createSingleVariable('kp', self.n_v)
        self.kd = self.prb.createSingleVariable('kd', self.n_v)

        self.q_init = self.prb.createSingleVariable('q_init', self.n_q)

        self.v_ee_param = self.prb.createParameter('v_ee_param', 1)
        self.impact = self.prb.createVariable('impact', 6, nodes=0)

        self.v_impact = cs.veccat(0., 0., - self.v_ee_param, 0., 0., 0.)

        # v0_min = 0 * np.ones(self.n_v)
        # v0_max = 100 * np.ones(self.n_v)
        # self.kp.setBounds(v0_min, v0_max)
        # self.kd.setBounds(v0_min, v0_max)

        return self.prb

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
