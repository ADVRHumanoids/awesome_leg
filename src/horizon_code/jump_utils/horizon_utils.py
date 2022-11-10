from horizon.utils import kin_dyn
import numpy as np

from horizon.utils import mat_storer

import matplotlib.pyplot as plt
# from matplotlib import cm
# from matplotlib.ticker import LinearLocator, FormatStrFormatter

import h5py

import warnings

def inv_dyn_from_sol(urdf_kin_dyn, q_p, q_p_dot, q_p_ddot, force_ref_frame, frame_force_map):

    ID = kin_dyn.InverseDynamics(urdf_kin_dyn, frame_force_map.keys(), force_ref_frame)

    n_intervals = q_p_ddot.shape[1]

    tau_res = np.zeros(q_p_ddot.shape)

    for i in range(n_intervals):
        frame_force_map_i = dict()
        for frame, wrench in frame_force_map.items():
            frame_force_map_i[frame] = wrench[:, i]
        tau_i = ID.call(q_p[:, i], q_p_dot[:, i], q_p_ddot[:, i], frame_force_map_i)
        tau_res[:, i] = tau_i.toarray().flatten()

    return tau_res


class JumpSolPlotter:

    def __init__(self, 
                solution_dirpath: str, 
                save_fullpath = "/tmp", 
                save_plots = False, 
                opt_base_sol_name = "awesome_jump",
                res_sol_suffix = "_res", 
                ref_sol_suffix = "_ref", 
                sim_postproc_filename = None, 
                test_postproc_filename = None):

        self.solution_path_raw = solution_dirpath + "/" + opt_base_sol_name
        self.solution_path_res = solution_dirpath + "/" + opt_base_sol_name + res_sol_suffix
        self.solution_path_ref = solution_dirpath + "/" + opt_base_sol_name + ref_sol_suffix
        
        self.__run_opt_sol_postproc()

        if sim_postproc_filename is not None:

            self.sim_path_full = solution_dirpath + "/" + sim_postproc_filename

            self.sim_data_loaded_ok = self.__run_sim_sol_postproc()
        
        if test_postproc_filename is not None:
            
            self.test_path_full = solution_dirpath + "/" + test_postproc_filename

            self.test_data_loaded_ok = self.__run_test_sol_postproc()
        

    def __run_opt_sol_postproc(self):

        self.__load_opt_sol()   

        self.__read_opt_sol()

        self.__compute_time_vect_opt()

    def __run_sim_sol_postproc(self):

        self.__load_sim_plugin_data()   

        sim_data_loaded_ok = self.__read_sim_plugin_data()

        return sim_data_loaded_ok

    def __run_test_sol_postproc(self):

        self.__load_test_plugin_data()   

        test_data_loaded_ok = self.__read_test_plugin_data()

        return test_data_loaded_ok
        
    def __load_opt_sol(self):
        
        self.ms_loader_raw = mat_storer.matStorer(self.solution_path_raw)
        self.ms_loader_res = mat_storer.matStorer(self.solution_path_res)
        self.ms_loader_ref = mat_storer.matStorer(self.solution_path_ref)
        
        try:

            self.solution_raw=self.ms_loader_raw.load() # loading the solution dictionary

            self.raw_opt_sol_read_ok = True

        except:
            
            print("\nFailed to load opt. solution data!!!\n")
            
            self.raw_opt_sol_read_ok = False

            pass

        try:

            self.solution_res=self.ms_loader_res.load() # loading the solution dictionary

            self.res_opt_sol_read_ok = True

        except:
            
            print("\nFailed to load resampled opt. solution data!!!\n")
            
            self.res_opt_sol_read_ok = False
    
            pass

        try:
            
            self.solution_ref=self.ms_loader_ref.load() # loading the solution dictionary

            self.ref_opt_sol_read_ok = True

        except:
            
            print("\nFailed to load refined opt. solution data!!!\n")

            self.ref_opt_sol_read_ok = False

            pass

    def __load_sim_plugin_data(self):
        
        self.ms_loader_sim = LogLoader(self.sim_path_full)
        self.sim_data = self.ms_loader_sim.data

    def __load_test_plugin_data(self):
        
        self.ms_loader_test = LogLoader(self.test_path_full)
        self.test_data = self.ms_loader_test.data

    def __read_raw_opt_sol(self):

        self.q_p_raw=self.solution_raw["q_p"] # excluding test rig dof
        self.q_p_dot_raw=self.solution_raw["q_p_dot"]
        self.q_p_ddot_raw=self.solution_raw["q_p_ddot"]
        self.i_q_raw=self.solution_raw["i_q"]
        self.GRF_raw=self.solution_raw["f_contact"]
        self.tau_raw=self.solution_raw["tau"][1:3,:] 
        self.dt_raw=self.solution_raw["dt_opt"].flatten()
        self.hip_height_raw=self.solution_raw["hip_height"]
        self.foot_tip_height_raw=self.solution_raw["foot_tip_height"]
        self.foot_tip_vel_raw=self.solution_raw["tip_velocity"]
        self.hip_vel_raw=self.solution_raw["hip_velocity"]

    def __read_res_opt_sol(self):
        
        self.q_p_res=self.solution_res["q_p"] # excluding test rig dof
        self.q_p_dot_res=self.solution_res["q_p_dot"]
        self.q_p_ddot_res=self.solution_res["q_p_ddot"]
        self.i_q_res=self.solution_res["i_q"]
        self.GRF_res=self.solution_res["f_contact"]
        self.tau_res=self.solution_res["tau"][1:3,:] 
        self.dt_res=self.solution_res["dt_opt"].flatten()
        self.hip_height_res=self.solution_res["hip_height"]
        self.foot_tip_height_res=self.solution_res["foot_tip_height"]
        self.foot_tip_vel_res=self.solution_res["tip_velocity"]
        self.hip_vel_res=self.solution_res["hip_velocity"]

    def __read_ref_opt_sol(self):

        self.q_p_ref=self.solution_ref["q_p"] # excluding test rig dof
        self.q_p_dot_ref=self.solution_ref["q_p_dot"]
        self.q_p_ddot_ref=self.solution_ref["q_p_ddot"]
        self.i_q_ref=self.solution_ref["i_q"]
        self.GRF_ref=self.solution_ref["f_contact"]
        self.tau_ref=self.solution_ref["tau"][1:3,:] 
        self.dt_ref=self.solution_res["dt_opt"].flatten() # refinement happend @ dt_res
        self.hip_height_ref=self.solution_ref["hip_height"]
        self.foot_tip_height_ref=self.solution_ref["foot_tip_height"]
        self.foot_tip_vel_ref=self.solution_ref["tip_velocity"]
        self.hip_vel_ref=self.solution_ref["hip_velocity"]

    def __read_opt_sol(self):
        
        if self.raw_opt_sol_read_ok:

            self.__read_raw_opt_sol()

        if self.res_opt_sol_read_ok:

            self.__read_res_opt_sol()

        if self.ref_opt_sol_read_ok:
            
            self.__read_ref_opt_sol()
    
    def __read_sim_plugin_data(self):

        self.is_sim=bool(self.sim_data["is_sim"][0][0])

        if self.is_sim:
            # the loaded data belongs to a simulation 
            self.sim_plugin_dt=self.sim_data["plugin_dt"] 
            self.sim_replay_time=self.sim_data["replay_time"]
            
            self.sim_q_p_meas=self.sim_data["q_p_meas"]
            self.sim_q_p_dot_meas=self.sim_data["q_p_dot_meas"]
            self.sim_q_p_cmd=self.sim_data["q_p_cmd"]
            self.sim_q_p_dot_cmd=self.sim_data["q_p_dot_cmd"]
            self.sim_replay_damping=self.sim_data["replay_damping"]
            self.sim_replay_stiffness=self.sim_data["replay_stiffness"]
            self.sim_meas_damping=self.sim_data["meas_damping"]
            self.sim_meas_stiffness=self.sim_data["meas_stiffness"]
            self.sim_stop_stiffness=self.sim_data["stop_stiffness"]
            self.sim_stop_damping=self.sim_data["stop_damping"]
            self.sim_tau_meas=self.sim_data["tau_meas"]
            self.sim_tau_cmd=self.sim_data["tau_cmd"]
            self.sim_base_link_abs=self.sim_data["base_link_abs"]
            self.sim_tip_pos_meas=self.sim_data["tip_pos_meas"]
            self.sim_tip_pos_rel_base_link=self.sim_data["tip_pos_rel_base_link"]

            self.sim_f_contact_ref = self.sim_data["f_contact_ref"]
            self.sim_meas_tip_f_loc = self.sim_data["meas_tip_f_loc"]
            self.sim_meas_tip_f_abs = self.sim_data["meas_tip_f_abs"]

            # other stuff

            self.is_sim=self.sim_data["is_sim"]
            self.is_sim=self.sim_data["send_pos_ref"]
            self.is_sim=self.sim_data["send_vel_ref"]
            self.is_sim=self.sim_data["send_eff_ref"]

            return True

        else:

            return False


    def __read_test_plugin_data(self):

        self.is_sim=bool(self.sim_data["is_sim"][0][0])
        
        if not self.is_sim:
            # the loaded data belongs to a test  
            self.sim_plugin_dt=self.sim_data["plugin_dt"] 
            self.sim_replay_time=self.sim_data["replay_time"]
            
            self.sim_q_p_meas=self.sim_data["q_p_meas"]
            self.sim_q_p_dot_meas=self.sim_data["q_p_dot_meas"]
            self.sim_q_p_cmd=self.sim_data["q_p_cmd"]
            self.sim_q_p_dot_cmd=self.sim_data["q_p_dot_cmd"]
            self.sim_replay_damping=self.sim_data["replay_damping"]
            self.sim_replay_stiffness=self.sim_data["replay_stiffness"]
            self.sim_meas_damping=self.sim_data["meas_damping"]
            self.sim_meas_stiffness=self.sim_data["meas_stiffness"]
            self.sim_stop_stiffness=self.sim_data["stop_stiffness"]
            self.sim_stop_damping=self.sim_data["stop_damping"]
            self.sim_tau_meas=self.sim_data["tau_meas"]
            self.sim_tau_cmd=self.sim_data["tau_cmd"]
        
            self.sim_tip_pos_rel_base_link=self.sim_data["tip_pos_rel_base_link"]

            self.sim_f_contact_ref = self.sim_data["f_contact_ref"]

            # other stuff

            self.is_sim=self.sim_data["is_sim"]
            self.is_sim=self.sim_data["send_pos_ref"]
            self.is_sim=self.sim_data["send_vel_ref"]
            self.is_sim=self.sim_data["send_eff_ref"]

            return True

        else:

            return False

    def __compute_time_vect_opt(self):
        
        if self.raw_opt_sol_read_ok:
            self.time_vector_raw = np.zeros([self.tau_raw[0, :].size + 1])
            for i in range(self.tau_raw[0,:].size):
                self.time_vector_raw[i+1] = self.time_vector_raw[i] + self.dt_raw[i]

        if self.res_opt_sol_read_ok:
            self.time_vector_res = np.zeros([self.tau_res[0, :].size + 1])
            for i in range(self.tau_res[0,:].size):
                self.time_vector_res[i+1] = self.time_vector_res[i] + self.dt_res[i]

        if self.ref_opt_sol_read_ok:
            self.time_vector_ref = np.zeros([self.tau_ref[0, :].size + 1])
            for i in range(self.tau_ref[0,:].size):
                self.time_vector_ref[i+1] = self.time_vector_ref[i] + self.dt_ref[i]

    def __make_raw_opt_plots(self):

        f1=plt.figure()
        plt.plot(self.time_vector_raw[1:-1].flatten(), self.GRF_raw[0, :-1], label=r"F_x")
        plt.plot(self.time_vector_raw[1:-1].flatten(), self.GRF_raw[1, :-1], label=r"F_y")
        plt.plot(self.time_vector_raw[1:-1].flatten(), self.GRF_raw[2, :-1], label=r"F_z")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel('[N]')
        plt.title("Ground reaction forces - raw solution", fontdict=None, loc='center')
        plt.grid()
        # if save_fig:
        #     plt.savefig(save_path+"GRF.pdf", format="pdf")

        f2=plt.figure()
        plt.plot(self.time_vector_raw[:-1].flatten(), self.q_p_raw[0, :-1], label=r"$\mathrm{q}_{\mathrm{hip}}$")
        plt.plot(self.time_vector_raw[:-1].flatten(), self.q_p_raw[1, :-1], label=r"$\mathrm{q}_{\mathrm{knee}}$")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[rad]")
        plt.title("Joint positions - raw solution", fontdict=None, loc='center')
        plt.grid()

        f3=plt.figure()
        plt.plot(self.time_vector_raw[:-1].flatten(), self.q_p_dot_raw[0, :-1], label=r"$\dot{q}_{\mathrm{hip}}$")
        plt.plot(self.time_vector_raw[:-1].flatten(), self.q_p_dot_raw[1, :-1], label=r"$\dot{q}_{\mathrm{knee}}$")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[rad/s]")
        plt.title("Joint velocities - raw solution", fontdict=None, loc='center')
        plt.grid()

        f4=plt.figure()
        plt.plot(self.time_vector_raw[:-1].flatten(), self.q_p_ddot_raw[0, :], label=r"$\ddot{\mathrm{q}}_{\mathrm{hip}}$")
        plt.plot(self.time_vector_raw[:-1].flatten(), self.q_p_ddot_raw[1, :], label=r"$\ddot{\mathrm{q}}_{\mathrm{knee}}$")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[$\mathrm{rad}/\mathrm{s}^2$]")
        plt.title("Joint accelerations - raw solution", fontdict=None, loc='center')
        plt.grid()
    
        f5=plt.figure()
        plt.plot(self.time_vector_raw[:-1].flatten(), self.tau_raw[0, :],  drawstyle='steps-post', label=r"$\tau_{\mathrm{hip}}$ torque")
        plt.plot(self.time_vector_raw[:-1].flatten(), self.tau_raw[1, :],  drawstyle='steps-post', label=r"$\tau_{\mathrm{knee}}$ torque")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[Nm]")
        plt.title("Joint efforts (link side) - raw solution", fontdict=None, loc='center')
        plt.grid()
        
        f6=plt.figure()
        plt.plot(self.time_vector_raw[:-1].flatten(), self.i_q_raw[0, :], label=r"$\mathrm{i}_\mathrm{q}^{\mathrm{hip}}$ current")
        plt.plot(self.time_vector_raw[:-1].flatten(), self.i_q_raw[1, :], label=r"$\mathrm{i}_\mathrm{q}^{\mathrm{knee}}$ current")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[A]")
        plt.title("Actuators quadrature current - raw solution", fontdict=None, loc='center')
        plt.grid()
    
        f8=plt.figure()
        plt.plot(self.time_vector_raw[:-1].flatten(), self.dt_raw, drawstyle='steps-post')
        plt.legend(loc="upper left")
        plt.ylabel(r"t [s]")
        plt.xlabel("node number")
        plt.title("Optimization dt - raw solution", fontdict=None, loc='center')
        plt.grid()

        f10=plt.figure()
        plt.plot(self.time_vector_raw.flatten(), self.hip_height_raw,label="hip height")
        plt.plot(self.time_vector_raw.flatten(), self.foot_tip_height_raw,label="foot tip height")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[m]")
        plt.title("Link heights during the jump - raw solution", fontdict=None, loc='center')
        plt.grid()
        
        f11=plt.figure()
        plt.plot(self.time_vector_raw.flatten(), self.foot_tip_vel_raw[0,:],label="tip vel. x")
        plt.plot(self.time_vector_raw.flatten(), self.foot_tip_vel_raw[1,:],label="tip vel. y")
        plt.plot(self.time_vector_raw.flatten(), self.foot_tip_vel_raw[2,:],label="tip vel. z")

        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[m/s]")
        plt.title("Foot tip velocity (cartesian components) - raw solution", fontdict=None, loc='center')
        plt.grid()
        
        f12=plt.figure()
        plt.plot(self.time_vector_raw.flatten(), self.hip_vel_raw[0,:],label="hip vel. x")
        plt.plot(self.time_vector_raw.flatten(), self.hip_vel_raw[1,:],label="hip vel. y")
        plt.plot(self.time_vector_raw.flatten(), self.hip_vel_raw[2,:],label="hip vel. z")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[m/s]")
        plt.title("Hip velocity (cartesian components) - raw solution", fontdict=None, loc='center')
        plt.grid()

    def __make_res_opt_plots(self):

        f1=plt.figure()
        plt.plot(self.time_vector_res[1:-1].flatten(), self.GRF_res[0, :-1], label=r"F_x")
        plt.plot(self.time_vector_res[1:-1].flatten(), self.GRF_res[1, :-1], label=r"F_y")
        plt.plot(self.time_vector_res[1:-1].flatten(), self.GRF_res[2, :-1], label=r"F_z")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel('[N]')
        plt.title("Ground reaction forces - res solution", fontdict=None, loc='center')
        plt.grid()
        # if save_fig:
        #     plt.savefig(save_path+"GRF.pdf", format="pdf")

        f2=plt.figure()
        plt.plot(self.time_vector_res[:-1].flatten(), self.q_p_res[0, :-1], label=r"$\mathrm{q}_{\mathrm{hip}}$")
        plt.plot(self.time_vector_res[:-1].flatten(), self.q_p_res[1, :-1], label=r"$\mathrm{q}_{\mathrm{knee}}$")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[rad]")
        plt.title("Joint positions - res solution", fontdict=None, loc='center')
        plt.grid()

        f3=plt.figure()
        plt.plot(self.time_vector_res[:-1].flatten(), self.q_p_dot_res[0, :-1], label=r"$\dot{q}_{\mathrm{hip}}$")
        plt.plot(self.time_vector_res[:-1].flatten(), self.q_p_dot_res[1, :-1], label=r"$\dot{q}_{\mathrm{knee}}$")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[rad/s]")
        plt.title("Joint velocities - res solution", fontdict=None, loc='center')
        plt.grid()

        f4=plt.figure()
        plt.plot(self.time_vector_res[:-1].flatten(), self.q_p_ddot_res[0, :], label=r"$\ddot{\mathrm{q}}_{\mathrm{hip}}$")
        plt.plot(self.time_vector_res[:-1].flatten(), self.q_p_ddot_res[1, :], label=r"$\ddot{\mathrm{q}}_{\mathrm{knee}}$")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[$\mathrm{rad}/\mathrm{s}^2$]")
        plt.title("Joint accelerations - res solution", fontdict=None, loc='center')
        plt.grid()
    
        f5=plt.figure()
        plt.plot(self.time_vector_res[:-1].flatten(), self.tau_res[0, :],  drawstyle='steps-post', label=r"$\tau_{\mathrm{hip}}$ torque")
        plt.plot(self.time_vector_res[:-1].flatten(), self.tau_res[1, :],  drawstyle='steps-post', label=r"$\tau_{\mathrm{knee}}$ torque")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[Nm]")
        plt.title("Joint efforts (link side) - res solution", fontdict=None, loc='center')
        plt.grid()
        
        f6=plt.figure()
        plt.plot(self.time_vector_res[:-1].flatten(), self.i_q_res[0, :], label=r"$\mathrm{i}_\mathrm{q}^{\mathrm{hip}}$ current")
        plt.plot(self.time_vector_res[:-1].flatten(), self.i_q_res[1, :], label=r"$\mathrm{i}_\mathrm{q}^{\mathrm{knee}}$ current")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[A]")
        plt.title("Actuators quadrature current - res solution", fontdict=None, loc='center')
        plt.grid()
    
        f8=plt.figure()
        plt.plot(self.time_vector_res[:-1].flatten(), self.dt_res, drawstyle='steps-post')
        plt.legend(loc="upper left")
        plt.ylabel(r"t [s]")
        plt.xlabel("node number")
        plt.title("Optimization dt - res solution", fontdict=None, loc='center')
        plt.grid()

        f10=plt.figure()
        plt.plot(self.time_vector_res.flatten(), self.hip_height_res,label="hip height")
        plt.plot(self.time_vector_res.flatten(), self.foot_tip_height_res,label="foot tip height")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[m]")
        plt.title("Link heights during the jump - res solution", fontdict=None, loc='center')
        plt.grid()
        
        f11=plt.figure()
        plt.plot(self.time_vector_res.flatten(), self.foot_tip_vel_res[0,:],label="tip vel. x")
        plt.plot(self.time_vector_res.flatten(), self.foot_tip_vel_res[1,:],label="tip vel. y")
        plt.plot(self.time_vector_res.flatten(), self.foot_tip_vel_res[2,:],label="tip vel. z")

        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[m/s]")
        plt.title("Foot tip velocity (cartesian components) - res solution", fontdict=None, loc='center')
        plt.grid()
        
        f12=plt.figure()
        plt.plot(self.time_vector_res.flatten(), self.hip_vel_res[0,:],label="hip vel. x")
        plt.plot(self.time_vector_res.flatten(), self.hip_vel_res[1,:],label="hip vel. y")
        plt.plot(self.time_vector_res.flatten(), self.hip_vel_res[2,:],label="hip vel. z")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[m/s]")
        plt.title("Hip velocity (cartesian components) - res solution", fontdict=None, loc='center')
        plt.grid()

    def __make_ref_opt_plots(self):

        f1=plt.figure()
        plt.plot(self.time_vector_ref[1:-1].flatten(), self.GRF_ref[0, :-1], label=r"F_x")
        plt.plot(self.time_vector_ref[1:-1].flatten(), self.GRF_ref[1, :-1], label=r"F_y")
        plt.plot(self.time_vector_ref[1:-1].flatten(), self.GRF_ref[2, :-1], label=r"F_z")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel('[N]')
        plt.title("Ground reaction forces - ref solution", fontdict=None, loc='center')
        plt.grid()
        # if save_fig:
        #     plt.savefig(save_path+"GRF.pdf", format="pdf")

        f2=plt.figure()
        plt.plot(self.time_vector_ref[:-1].flatten(), self.q_p_ref[0, :-1], label=r"$\mathrm{q}_{\mathrm{hip}}$")
        plt.plot(self.time_vector_ref[:-1].flatten(), self.q_p_ref[1, :-1], label=r"$\mathrm{q}_{\mathrm{knee}}$")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[rad]")
        plt.title("Joint positions - ref solution", fontdict=None, loc='center')
        plt.grid()

        f3=plt.figure()
        plt.plot(self.time_vector_ref[:-1].flatten(), self.q_p_dot_ref[0, :-1], label=r"$\dot{q}_{\mathrm{hip}}$")
        plt.plot(self.time_vector_ref[:-1].flatten(), self.q_p_dot_ref[1, :-1], label=r"$\dot{q}_{\mathrm{knee}}$")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[rad/s]")
        plt.title("Joint velocities - ref solution", fontdict=None, loc='center')
        plt.grid()

        f4=plt.figure()
        plt.plot(self.time_vector_ref[:-1].flatten(), self.q_p_ddot_ref[0, :], label=r"$\ddot{\mathrm{q}}_{\mathrm{hip}}$")
        plt.plot(self.time_vector_ref[:-1].flatten(), self.q_p_ddot_ref[1, :], label=r"$\ddot{\mathrm{q}}_{\mathrm{knee}}$")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[$\mathrm{rad}/\mathrm{s}^2$]")
        plt.title("Joint accelerations - ref solution", fontdict=None, loc='center')
        plt.grid()
    
        f5=plt.figure()
        plt.plot(self.time_vector_ref[:-1].flatten(), self.tau_ref[0, :],  drawstyle='steps-post', label=r"$\tau_{\mathrm{hip}}$ torque")
        plt.plot(self.time_vector_ref[:-1].flatten(), self.tau_ref[1, :],  drawstyle='steps-post', label=r"$\tau_{\mathrm{knee}}$ torque")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[Nm]")
        plt.title("Joint efforts (link side) - ref solution", fontdict=None, loc='center')
        plt.grid()
        
        f6=plt.figure()
        plt.plot(self.time_vector_ref[:-1].flatten(), self.i_q_ref[0, :], label=r"$\mathrm{i}_\mathrm{q}^{\mathrm{hip}}$ current")
        plt.plot(self.time_vector_ref[:-1].flatten(), self.i_q_ref[1, :], label=r"$\mathrm{i}_\mathrm{q}^{\mathrm{knee}}$ current")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[A]")
        plt.title("Actuators quadrature current - ref solution", fontdict=None, loc='center')
        plt.grid()
    
        f8=plt.figure()
        plt.plot(self.time_vector_ref[:-1].flatten(), self.dt_ref, drawstyle='steps-post')
        plt.legend(loc="upper left")
        plt.ylabel(r"t [s]")
        plt.xlabel("node number")
        plt.title("Optimization dt - ref solution", fontdict=None, loc='center')
        plt.grid()

        f10=plt.figure()
        plt.plot(self.time_vector_ref.flatten(), self.hip_height_ref,label="hip height")
        plt.plot(self.time_vector_ref.flatten(), self.foot_tip_height_ref,label="foot tip height")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[m]")
        plt.title("Link heights during the jump - ref solution", fontdict=None, loc='center')
        plt.grid()
        
        f11=plt.figure()
        plt.plot(self.time_vector_ref.flatten(), self.foot_tip_vel_ref[0,:],label="tip vel. x")
        plt.plot(self.time_vector_ref.flatten(), self.foot_tip_vel_ref[1,:],label="tip vel. y")
        plt.plot(self.time_vector_ref.flatten(), self.foot_tip_vel_ref[2,:],label="tip vel. z")

        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[m/s]")
        plt.title("Foot tip velocity (cartesian components) - ref solution", fontdict=None, loc='center')
        plt.grid()
        
        f12=plt.figure()
        plt.plot(self.time_vector_ref.flatten(), self.hip_vel_ref[0,:],label="hip vel. x")
        plt.plot(self.time_vector_ref.flatten(), self.hip_vel_ref[1,:],label="hip vel. y")
        plt.plot(self.time_vector_ref.flatten(), self.hip_vel_ref[2,:],label="hip vel. z")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[m/s]")
        plt.title("Hip velocity (cartesian components) - ref solution", fontdict=None, loc='center')
        plt.grid()

    def __make_rawref_compar_plots(self):

        f1=plt.figure()

        plt.plot(self.time_vector_raw[1:-1].flatten(), self.GRF_raw[0, :-1], label=r"F_x_raw", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_raw[1:-1].flatten(), self.GRF_raw[1, :-1], label=r"F_y_raw", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_raw[1:-1].flatten(), self.GRF_raw[2, :-1], label=r"F_z_raw", linestyle='-', linewidth=2)

        plt.plot(self.time_vector_ref[1:-1].flatten(), self.GRF_ref[0, :-1], label=r"F_x_ref", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_ref[1:-1].flatten(), self.GRF_ref[1, :-1], label=r"F_y_ref", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_ref[1:-1].flatten(), self.GRF_ref[2, :-1], label=r"F_z_ref", linestyle='dashed', linewidth=2)

        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel('[N]')
        plt.title("Ground reaction forces - raw VS ref", fontdict=None, loc='center')
        plt.grid()
        # if save_fig:
        #     plt.savefig(save_path+"GRF.pdf", format="pdf")

        f2=plt.figure()
        plt.plot(self.time_vector_raw[:-1].flatten(), self.q_p_raw[0, :-1], label=r"$\mathrm{q}_{\mathrm{hip}}$_raw", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_raw[:-1].flatten(), self.q_p_raw[1, :-1], label=r"$\mathrm{q}_{\mathrm{knee}}$_raw", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_ref[:-1].flatten(), self.q_p_ref[0, :-1], label=r"$\mathrm{q}_{\mathrm{hip}}$_ref", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_ref[:-1].flatten(), self.q_p_ref[1, :-1], label=r"$\mathrm{q}_{\mathrm{knee}}$_ref", linestyle='dashed', linewidth=2)
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[rad]")
        plt.title("Joint positions - raw VS ref", fontdict=None, loc='center')
        plt.grid()

        f3=plt.figure()
        plt.plot(self.time_vector_raw[:-1].flatten(), self.q_p_dot_raw[0, :-1], label=r"$\dot{q}_{\mathrm{hip}}$_raw", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_raw[:-1].flatten(), self.q_p_dot_raw[1, :-1], label=r"$\dot{q}_{\mathrm{knee}}_raw$", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_ref[:-1].flatten(), self.q_p_dot_ref[0, :-1], label=r"$\dot{q}_{\mathrm{hip}}$_ref", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_ref[:-1].flatten(), self.q_p_dot_ref[1, :-1], label=r"$\dot{q}_{\mathrm{knee}}$_ref", linestyle='dashed', linewidth=2)
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[rad/s]")
        plt.title("Joint velocities - raw VS ref", fontdict=None, loc='center')
        plt.grid()

        f4=plt.figure()
        plt.plot(self.time_vector_raw[:-1].flatten(), self.q_p_ddot_raw[0, :], label=r"$\ddot{\mathrm{q}}_{\mathrm{hip}}$_raw", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_raw[:-1].flatten(), self.q_p_ddot_raw[1, :], label=r"$\ddot{\mathrm{q}}_{\mathrm{knee}}$_raw", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_ref[:-1].flatten(), self.q_p_ddot_ref[0, :], label=r"$\ddot{\mathrm{q}}_{\mathrm{hip}}$_ref", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_ref[:-1].flatten(), self.q_p_ddot_ref[1, :], label=r"$\ddot{\mathrm{q}}_{\mathrm{knee}}$_ref", linestyle='dashed', linewidth=2)
        
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[$\mathrm{rad}/\mathrm{s}^2$]")
        plt.title("Joint accelerations - raw VS ref", fontdict=None, loc='center')
        plt.grid()
    
        f5=plt.figure()
        plt.plot(self.time_vector_raw[:-1].flatten(), self.tau_raw[0, :],  drawstyle='steps-post', label=r"$\tau_{\mathrm{hip}}$ torque raw", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_raw[:-1].flatten(), self.tau_raw[1, :],  drawstyle='steps-post', label=r"$\tau_{\mathrm{knee}}$ torque raw", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_ref[:-1].flatten(), self.tau_ref[0, :],  drawstyle='steps-post', label=r"$\tau_{\mathrm{hip}}$ torque ref", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_ref[:-1].flatten(), self.tau_ref[1, :],  drawstyle='steps-post', label=r"$\tau_{\mathrm{knee}}$ torque ref", linestyle='dashed', linewidth=2)
        
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[Nm]")
        plt.title("Joint efforts (link side) - raw VS ref", fontdict=None, loc='center')
        plt.grid()
        
        f6=plt.figure()
        plt.plot(self.time_vector_raw[:-1].flatten(), self.i_q_raw[0, :], label=r"$\mathrm{i}_\mathrm{q}^{\mathrm{hip}}$ current raw", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_raw[:-1].flatten(), self.i_q_raw[1, :], label=r"$\mathrm{i}_\mathrm{q}^{\mathrm{knee}}$ current raw", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_ref[:-1].flatten(), self.i_q_ref[0, :], label=r"$\mathrm{i}_\mathrm{q}^{\mathrm{hip}}$ current ref", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_ref[:-1].flatten(), self.i_q_ref[1, :], label=r"$\mathrm{i}_\mathrm{q}^{\mathrm{knee}}$ current ref", linestyle='dashed', linewidth=2)
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[A]")
        plt.title("Actuators quadrature current - raw VS ref", fontdict=None, loc='center')
        plt.grid()

        f10=plt.figure()
        plt.plot(self.time_vector_raw.flatten(), self.hip_height_raw,label="hip height raw", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_raw.flatten(), self.foot_tip_height_raw,label="foot tip height raw", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_ref.flatten(), self.hip_height_ref,label="hip height ref", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_ref.flatten(), self.foot_tip_height_ref,label="foot tip height ref", linestyle='dashed', linewidth=2)
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[m]")
        plt.title("Link heights during the jump - raw VS ref", fontdict=None, loc='center')
        plt.grid()
        
        f11=plt.figure()
        plt.plot(self.time_vector_raw.flatten(), self.foot_tip_vel_raw[0,:],label="tip vel. x raw", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_raw.flatten(), self.foot_tip_vel_raw[1,:],label="tip vel. y raw", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_raw.flatten(), self.foot_tip_vel_raw[2,:],label="tip vel. z raw", linestyle='-', linewidth=2)

        plt.plot(self.time_vector_ref.flatten(), self.foot_tip_vel_ref[0,:],label="tip vel. x ref", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_ref.flatten(), self.foot_tip_vel_ref[1,:],label="tip vel. y ref", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_ref.flatten(), self.foot_tip_vel_ref[2,:],label="tip vel. z ref", linestyle='dashed', linewidth=2)

        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[m/s]")
        plt.title("Foot tip velocity (cartesian components) - raw VS ref", fontdict=None, loc='center')
        plt.grid()
        
        f12=plt.figure()
        plt.plot(self.time_vector_raw.flatten(), self.hip_vel_raw[0,:],label="hip vel. x raw", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_raw.flatten(), self.hip_vel_raw[1,:],label="hip vel. y raw", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_raw.flatten(), self.hip_vel_raw[2,:],label="hip vel. z raw", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_ref.flatten(), self.hip_vel_ref[0,:],label="hip vel. x ref", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_ref.flatten(), self.hip_vel_ref[1,:],label="hip vel. y ref", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_ref.flatten(), self.hip_vel_ref[2,:],label="hip vel. z ref", linestyle='dashed', linewidth=2)
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[m/s]")
        plt.title("Hip velocity (cartesian components) - raw VS ref", fontdict=None, loc='center')
        plt.grid()

    def __make_resref_compar_plots(self):

        f1=plt.figure()

        plt.plot(self.time_vector_res[1:-1].flatten(), self.GRF_res[0, :-1], label=r"F_x_res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_res[1:-1].flatten(), self.GRF_res[1, :-1], label=r"F_y_res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_res[1:-1].flatten(), self.GRF_res[2, :-1], label=r"F_z_res", linestyle='-', linewidth=2)

        plt.plot(self.time_vector_ref[1:-1].flatten(), self.GRF_ref[0, :-1], label=r"F_x_ref", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_ref[1:-1].flatten(), self.GRF_ref[1, :-1], label=r"F_y_ref", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_ref[1:-1].flatten(), self.GRF_ref[2, :-1], label=r"F_z_ref", linestyle='dashed', linewidth=2)

        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel('[N]')
        plt.title("Ground reaction forces - res VS ref", fontdict=None, loc='center')
        plt.grid()
        # if save_fig:
        #     plt.savefig(save_path+"GRF.pdf", format="pdf")

        f2=plt.figure()
        plt.plot(self.time_vector_res[:-1].flatten(), self.q_p_res[0, :-1], label=r"$\mathrm{q}_{\mathrm{hip}}$_res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_res[:-1].flatten(), self.q_p_res[1, :-1], label=r"$\mathrm{q}_{\mathrm{knee}}$_res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_ref[:-1].flatten(), self.q_p_ref[0, :-1], label=r"$\mathrm{q}_{\mathrm{hip}}$_ref", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_ref[:-1].flatten(), self.q_p_ref[1, :-1], label=r"$\mathrm{q}_{\mathrm{knee}}$_ref", linestyle='dashed', linewidth=2)
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[rad]")
        plt.title("Joint positions - res VS ref", fontdict=None, loc='center')
        plt.grid()

        f3=plt.figure()
        plt.plot(self.time_vector_res[:-1].flatten(), self.q_p_dot_res[0, :-1], label=r"$\dot{q}_{\mathrm{hip}}$_res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_res[:-1].flatten(), self.q_p_dot_res[1, :-1], label=r"$\dot{q}_{\mathrm{knee}}_res$", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_ref[:-1].flatten(), self.q_p_dot_ref[0, :-1], label=r"$\dot{q}_{\mathrm{hip}}$_ref", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_ref[:-1].flatten(), self.q_p_dot_ref[1, :-1], label=r"$\dot{q}_{\mathrm{knee}}$_ref", linestyle='dashed', linewidth=2)
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[rad/s]")
        plt.title("Joint velocities - res VS ref", fontdict=None, loc='center')
        plt.grid()

        f4=plt.figure()
        plt.plot(self.time_vector_res[:-1].flatten(), self.q_p_ddot_res[0, :], label=r"$\ddot{\mathrm{q}}_{\mathrm{hip}}$_res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_res[:-1].flatten(), self.q_p_ddot_res[1, :], label=r"$\ddot{\mathrm{q}}_{\mathrm{knee}}$_res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_ref[:-1].flatten(), self.q_p_ddot_ref[0, :], label=r"$\ddot{\mathrm{q}}_{\mathrm{hip}}$_ref", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_ref[:-1].flatten(), self.q_p_ddot_ref[1, :], label=r"$\ddot{\mathrm{q}}_{\mathrm{knee}}$_ref", linestyle='dashed', linewidth=2)
        
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[$\mathrm{rad}/\mathrm{s}^2$]")
        plt.title("Joint accelerations - res VS ref", fontdict=None, loc='center')
        plt.grid()
    
        f5=plt.figure()
        plt.plot(self.time_vector_res[:-1].flatten(), self.tau_res[0, :],  drawstyle='steps-post', label=r"$\tau_{\mathrm{hip}}$ torque res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_res[:-1].flatten(), self.tau_res[1, :],  drawstyle='steps-post', label=r"$\tau_{\mathrm{knee}}$ torque res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_ref[:-1].flatten(), self.tau_ref[0, :],  drawstyle='steps-post', label=r"$\tau_{\mathrm{hip}}$ torque ref", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_ref[:-1].flatten(), self.tau_ref[1, :],  drawstyle='steps-post', label=r"$\tau_{\mathrm{knee}}$ torque ref", linestyle='dashed', linewidth=2)
        
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[Nm]")
        plt.title("Joint efforts (link side) - res VS ref", fontdict=None, loc='center')
        plt.grid()
        
        f6=plt.figure()
        plt.plot(self.time_vector_res[:-1].flatten(), self.i_q_res[0, :], label=r"$\mathrm{i}_\mathrm{q}^{\mathrm{hip}}$ current res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_res[:-1].flatten(), self.i_q_res[1, :], label=r"$\mathrm{i}_\mathrm{q}^{\mathrm{knee}}$ current res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_ref[:-1].flatten(), self.i_q_ref[0, :], label=r"$\mathrm{i}_\mathrm{q}^{\mathrm{hip}}$ current ref", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_ref[:-1].flatten(), self.i_q_ref[1, :], label=r"$\mathrm{i}_\mathrm{q}^{\mathrm{knee}}$ current ref", linestyle='dashed', linewidth=2)
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[A]")
        plt.title("Actuators quadrature current - res VS ref", fontdict=None, loc='center')
        plt.grid()

        f10=plt.figure()
        plt.plot(self.time_vector_res.flatten(), self.hip_height_res,label="hip height res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_res.flatten(), self.foot_tip_height_res,label="foot tip height res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_ref.flatten(), self.hip_height_ref,label="hip height ref", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_ref.flatten(), self.foot_tip_height_ref,label="foot tip height ref", linestyle='dashed', linewidth=2)
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[m]")
        plt.title("Link heights during the jump - res VS ref", fontdict=None, loc='center')
        plt.grid()
        
        f11=plt.figure()
        plt.plot(self.time_vector_res.flatten(), self.foot_tip_vel_res[0,:],label="tip vel. x res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_res.flatten(), self.foot_tip_vel_res[1,:],label="tip vel. y res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_res.flatten(), self.foot_tip_vel_res[2,:],label="tip vel. z res", linestyle='-', linewidth=2)

        plt.plot(self.time_vector_ref.flatten(), self.foot_tip_vel_ref[0,:],label="tip vel. x ref", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_ref.flatten(), self.foot_tip_vel_ref[1,:],label="tip vel. y ref", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_ref.flatten(), self.foot_tip_vel_ref[2,:],label="tip vel. z ref", linestyle='dashed', linewidth=2)

        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[m/s]")
        plt.title("Foot tip velocity (cartesian components) - res VS ref", fontdict=None, loc='center')
        plt.grid()
        
        f12=plt.figure()
        plt.plot(self.time_vector_res.flatten(), self.hip_vel_res[0,:],label="hip vel. x res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_res.flatten(), self.hip_vel_res[1,:],label="hip vel. y res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_res.flatten(), self.hip_vel_res[2,:],label="hip vel. z res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_ref.flatten(), self.hip_vel_ref[0,:],label="hip vel. x ref", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_ref.flatten(), self.hip_vel_ref[1,:],label="hip vel. y ref", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_ref.flatten(), self.hip_vel_ref[2,:],label="hip vel. z ref", linestyle='dashed', linewidth=2)
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[m/s]")
        plt.title("Hip velocity (cartesian components) - res VS ref", fontdict=None, loc='center')
        plt.grid()
    
    def __make_rawres_compar_plots(self):

        f1=plt.figure()

        plt.plot(self.time_vector_res[1:-1].flatten(), self.GRF_res[0, :-1], label=r"F_x_res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_res[1:-1].flatten(), self.GRF_res[1, :-1], label=r"F_y_res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_res[1:-1].flatten(), self.GRF_res[2, :-1], label=r"F_z_res", linestyle='-', linewidth=2)

        plt.plot(self.time_vector_raw[1:-1].flatten(), self.GRF_raw[0, :-1], label=r"F_x_raw", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_raw[1:-1].flatten(), self.GRF_raw[1, :-1], label=r"F_y_raw", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_raw[1:-1].flatten(), self.GRF_raw[2, :-1], label=r"F_z_raw", linestyle='dashed', linewidth=2)

        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel('[N]')
        plt.title("Ground reaction forces - res VS raw", fontdict=None, loc='center')
        plt.grid()
        # if save_fig:
        #     plt.savefig(save_path+"GRF.pdf", format="pdf")

        f2=plt.figure()
        plt.plot(self.time_vector_res[:-1].flatten(), self.q_p_res[0, :-1], label=r"$\mathrm{q}_{\mathrm{hip}}$_res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_res[:-1].flatten(), self.q_p_res[1, :-1], label=r"$\mathrm{q}_{\mathrm{knee}}$_res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_raw[:-1].flatten(), self.q_p_raw[0, :-1], label=r"$\mathrm{q}_{\mathrm{hip}}$_raw", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_raw[:-1].flatten(), self.q_p_raw[1, :-1], label=r"$\mathrm{q}_{\mathrm{knee}}$_raw", linestyle='dashed', linewidth=2)
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[rad]")
        plt.title("Joint positions - res VS raw", fontdict=None, loc='center')
        plt.grid()

        f3=plt.figure()
        plt.plot(self.time_vector_res[:-1].flatten(), self.q_p_dot_res[0, :-1], label=r"$\dot{q}_{\mathrm{hip}}$_res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_res[:-1].flatten(), self.q_p_dot_res[1, :-1], label=r"$\dot{q}_{\mathrm{knee}}_res$", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_raw[:-1].flatten(), self.q_p_dot_raw[0, :-1], label=r"$\dot{q}_{\mathrm{hip}}$_raw", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_raw[:-1].flatten(), self.q_p_dot_raw[1, :-1], label=r"$\dot{q}_{\mathrm{knee}}$_raw", linestyle='dashed', linewidth=2)
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[rad/s]")
        plt.title("Joint velocities - res VS raw", fontdict=None, loc='center')
        plt.grid()

        f4=plt.figure()
        plt.plot(self.time_vector_res[:-1].flatten(), self.q_p_ddot_res[0, :], label=r"$\ddot{\mathrm{q}}_{\mathrm{hip}}$_res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_res[:-1].flatten(), self.q_p_ddot_res[1, :], label=r"$\ddot{\mathrm{q}}_{\mathrm{knee}}$_res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_raw[:-1].flatten(), self.q_p_ddot_raw[0, :], label=r"$\ddot{\mathrm{q}}_{\mathrm{hip}}$_raw", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_raw[:-1].flatten(), self.q_p_ddot_raw[1, :], label=r"$\ddot{\mathrm{q}}_{\mathrm{knee}}$_raw", linestyle='dashed', linewidth=2)
        
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[$\mathrm{rad}/\mathrm{s}^2$]")
        plt.title("Joint accelerations - res VS raw", fontdict=None, loc='center')
        plt.grid()
    
        f5=plt.figure()
        plt.plot(self.time_vector_res[:-1].flatten(), self.tau_res[0, :],  drawstyle='steps-post', label=r"$\tau_{\mathrm{hip}}$ torque res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_res[:-1].flatten(), self.tau_res[1, :],  drawstyle='steps-post', label=r"$\tau_{\mathrm{knee}}$ torque res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_raw[:-1].flatten(), self.tau_raw[0, :],  drawstyle='steps-post', label=r"$\tau_{\mathrm{hip}}$ torque raw", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_raw[:-1].flatten(), self.tau_raw[1, :],  drawstyle='steps-post', label=r"$\tau_{\mathrm{knee}}$ torque raw", linestyle='dashed', linewidth=2)
        
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[Nm]")
        plt.title("Joint efforts (link side) - res VS raw", fontdict=None, loc='center')
        plt.grid()
        
        f6=plt.figure()
        plt.plot(self.time_vector_res[:-1].flatten(), self.i_q_res[0, :], label=r"$\mathrm{i}_\mathrm{q}^{\mathrm{hip}}$ current res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_res[:-1].flatten(), self.i_q_res[1, :], label=r"$\mathrm{i}_\mathrm{q}^{\mathrm{knee}}$ current res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_raw[:-1].flatten(), self.i_q_raw[0, :], label=r"$\mathrm{i}_\mathrm{q}^{\mathrm{hip}}$ current raw", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_raw[:-1].flatten(), self.i_q_raw[1, :], label=r"$\mathrm{i}_\mathrm{q}^{\mathrm{knee}}$ current raw", linestyle='dashed', linewidth=2)
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[A]")
        plt.title("Actuators quadrature current - res VS raw", fontdict=None, loc='center')
        plt.grid()

        f10=plt.figure()
        plt.plot(self.time_vector_res.flatten(), self.hip_height_res,label="hip height res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_res.flatten(), self.foot_tip_height_res,label="foot tip height res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_raw.flatten(), self.hip_height_raw,label="hip height raw", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_raw.flatten(), self.foot_tip_height_raw,label="foot tip height raw", linestyle='dashed', linewidth=2)
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[m]")
        plt.title("Link heights during the jump - res VS raw", fontdict=None, loc='center')
        plt.grid()
        
        f11=plt.figure()
        plt.plot(self.time_vector_res.flatten(), self.foot_tip_vel_res[0,:],label="tip vel. x res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_res.flatten(), self.foot_tip_vel_res[1,:],label="tip vel. y res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_res.flatten(), self.foot_tip_vel_res[2,:],label="tip vel. z res", linestyle='-', linewidth=2)

        plt.plot(self.time_vector_raw.flatten(), self.foot_tip_vel_raw[0,:],label="tip vel. x raw", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_raw.flatten(), self.foot_tip_vel_raw[1,:],label="tip vel. y raw", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_raw.flatten(), self.foot_tip_vel_raw[2,:],label="tip vel. z raw", linestyle='dashed', linewidth=2)

        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[m/s]")
        plt.title("Foot tip velocity (cartesian components) - res VS raw", fontdict=None, loc='center')
        plt.grid()
        
        f12=plt.figure()
        plt.plot(self.time_vector_res.flatten(), self.hip_vel_res[0,:],label="hip vel. x res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_res.flatten(), self.hip_vel_res[1,:],label="hip vel. y res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_res.flatten(), self.hip_vel_res[2,:],label="hip vel. z res", linestyle='-', linewidth=2)
        plt.plot(self.time_vector_raw.flatten(), self.hip_vel_raw[0,:],label="hip vel. x raw", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_raw.flatten(), self.hip_vel_raw[1,:],label="hip vel. y raw", linestyle='dashed', linewidth=2)
        plt.plot(self.time_vector_raw.flatten(), self.hip_vel_raw[2,:],label="hip vel. z raw", linestyle='dashed', linewidth=2)
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[m/s]")
        plt.title("Hip velocity (cartesian components) - res VS raw", fontdict=None, loc='center')
        plt.grid()

    def make_opt_plots(self, make_plt_slctr = [True] * 6):

        if make_plt_slctr[0]:
            self.__make_raw_opt_plots()
        
        if make_plt_slctr[1]:
            self.__make_res_opt_plots()
        
        if make_plt_slctr[2]:
            self.__make_ref_opt_plots()

        if make_plt_slctr[3]:
            self.__make_resref_compar_plots()

        if make_plt_slctr[4]:
            self.__make_rawres_compar_plots()

        if make_plt_slctr[5]:
            self.__make_rawref_compar_plots()

        return True

    def make_sim_plots(self, make_plots = False):
        
        if make_plots and self.sim_data_loaded_ok:

            _, ax_pos = plt.subplots(2)

            for i in range(len(self.sim_q_p_meas[:, 0])):
                
                print
                ax_pos[0].plot(self.sim_replay_time.flatten(),self.sim_q_p_meas[i, :], label=r"q_meas_j" + str(i), linestyle='-', linewidth=2)
                ax_pos[0].plot(self.sim_replay_time.flatten(), self.sim_q_p_cmd[i, :], label=r"q_cmd_j" + str(i), linestyle='dashed', linewidth=2)

                # errors
                ax_pos[1].plot(self.sim_replay_time.flatten(), np.subtract(self.sim_q_p_cmd[i, :], self.sim_q_p_meas[i, :]),\
                    label=r"q_err_j" + str(i), linestyle='-', linewidth=2)
            
            ax_pos[0].legend(loc="upper left")
            ax_pos[0].set_xlabel(r"t [s]")
            ax_pos[0].set_ylabel('[rad]')
            ax_pos[0].grid()
            ax_pos[0].set_title("Joint positions - meas. VS ref.", fontdict=None, loc='center')
            ax_pos[1].legend(loc="upper left")
            ax_pos[1].set_xlabel(r"t [s]")
            ax_pos[1].set_ylabel('[rad]')
            ax_pos[1].grid()
            ax_pos[1].set_title("Joint pos. tracking error", fontdict=None, loc='center')
            
            _, ax_vel = plt.subplots(2)

            for i in range(len(self.sim_q_p_meas[:, 0])):
                
                ax_vel[0].plot(self.sim_replay_time.flatten(), self.sim_q_p_dot_meas[i, :], label=r"q_meas_j" + str(i), linestyle='-', linewidth=2)
                ax_vel[0].plot(self.sim_replay_time.flatten(), self.sim_q_p_dot_cmd[i, :], label=r"q_cmd_j" + str(i), linestyle='dashed', linewidth=2)

                # errors
                ax_vel[1].plot(self.sim_replay_time.flatten(), np.subtract(self.sim_q_p_dot_cmd[i, :],\
                    self.sim_q_p_dot_meas[i, :]), label=r"q_err_j" + str(i), linestyle='-', linewidth=2)
                
            ax_vel[0].legend(loc="upper left")
            ax_vel[0].set_xlabel(r"t [s]")
            ax_vel[0].set_ylabel('[rad]')
            ax_vel[0].grid()
            ax_vel[0].set_title("Joint velocities - meas. VS ref.", fontdict=None, loc='center')
            ax_vel[1].legend(loc="upper left")
            ax_vel[1].set_xlabel(r"t [s]")
            ax_vel[1].set_ylabel('[rad/s]')
            ax_vel[1].grid()
            ax_vel[1].set_title("Joint vel. tracking error", fontdict=None, loc='center')

            f2=plt.figure()

            for i in range(len(self.sim_tau_meas[:, 0])):

                plt.plot(self.sim_replay_time.flatten(), self.sim_tau_meas[i, :], label=r"tau_meas_j" + str(i), linestyle='-', linewidth=2)

                plt.plot(self.sim_replay_time.flatten(), self.sim_tau_cmd[i, :], label=r"tau_cmd_j" + str(i), linestyle='dashed', linewidth=2)

            plt.legend(loc="upper left")
            plt.xlabel(r"t [s]")
            plt.ylabel('[Nm]')
            plt.title("Joint efforts - meas. VS ref.", fontdict=None, loc='center')
            plt.grid()

            f3=plt.figure()
            for i in range(len(self.sim_tau_meas[:, 0])):

                plt.plot(self.sim_replay_time.flatten(), self.sim_tau_meas[i, :] * self.sim_q_p_dot_meas[i, :], label=r"mech_pow_meas_j" + str(i), linestyle='-', linewidth=2)

                plt.plot(self.sim_replay_time.flatten(), self.sim_tau_cmd[i, :] * self.sim_q_p_dot_cmd[i, :], label=r"mech_pow_ref_j" + str(i), linestyle='dashed', linewidth=2)

            plt.legend(loc="upper left")
            plt.xlabel(r"t [s]")
            plt.ylabel('[m]')
            plt.title("Joint mechanical power - meas. VS ref.", fontdict=None, loc='center')
            plt.grid()

            _, ax_imp = plt.subplots(2)

            for i in range(len(self.sim_meas_stiffness[:, 0])):
                
                ax_imp[0].plot(self.sim_replay_time.flatten(), self.sim_meas_stiffness[i, :], label=r"read jnt stiffness" + str(i), linestyle='-', linewidth=2)

                ax_imp[1].plot(self.sim_replay_time.flatten(), self.sim_meas_damping[i, :], label=r"read jnt damping" + str(i), linestyle='-', linewidth=2)
                
            ax_imp[0].legend(loc="upper left")
            ax_imp[0].set_xlabel(r"t [s]")
            ax_imp[0].set_ylabel('[Nm / rad]')
            ax_imp[0].grid()
            ax_imp[0].set_title("Read joint stiffnesses", fontdict=None, loc='center')
            ax_imp[1].legend(loc="upper left")
            ax_imp[1].set_xlabel(r"t [s]")
            ax_imp[1].set_ylabel('[Nm s / rad]')
            ax_imp[1].grid()
            ax_imp[1].set_title("Read joint dampings", fontdict=None, loc='center')

            _, ax_f_cont_meas = plt.subplots(2)

            directions = ["x", "y", "z"]

            for i in range(len(self.sim_meas_tip_f_abs[:, 0])): # iterate through spatial directions
                
                ax_f_cont_meas[0].plot(self.sim_replay_time.flatten(), self.sim_meas_tip_f_abs[i, :], label=r"contact force along abs. " + directions[i] + r" direction", linestyle='-', linewidth=2)

                ax_f_cont_meas[1].plot(self.sim_replay_time.flatten(), self.sim_meas_tip_f_loc[i, :], label=r"contact force along loc. " + directions[i] + r" direction", linestyle='-', linewidth=2)
                
            ax_f_cont_meas[0].legend(loc="upper left")
            ax_f_cont_meas[0].set_xlabel(r"t [s]")
            ax_f_cont_meas[0].set_ylabel('[N]')
            ax_f_cont_meas[0].grid()
            ax_f_cont_meas[0].set_title("Measured absolute contact forces", fontdict=None, loc='center')
            ax_f_cont_meas[1].legend(loc="upper left")
            ax_f_cont_meas[1].set_xlabel(r"t [s]")
            ax_f_cont_meas[1].set_ylabel('[N]')
            ax_f_cont_meas[1].grid()
            ax_f_cont_meas[1].set_title("Measured local (tip frame) contact forces", fontdict=None, loc='center')
            
            self.__make_sim_link_plots()
            
    def __make_sim_link_plots(self):
                
        _, ax_sol_t = plt.subplots(2)

        ax_sol_t[0].plot(self.sim_replay_time.flatten(), self.sim_base_link_abs[2, :], label = "$z_{hip}$",\
            linestyle='-', linewidth=2, markersize=12)
        leg_t = ax_sol_t[0].legend(loc="upper right")
        leg_t.set_draggable(True)
        # ax_sol_t[0].set_xlabel(r"time [s]")
        ax_sol_t[0].set_ylabel(r"hip pos [m]")
        ax_sol_t[0].set_title(r"Hip absolute vertical position", fontdict=None, loc='center')
        ax_sol_t[0].grid()

        ax_sol_t[1].plot(self.sim_replay_time.flatten(), self.sim_tip_pos_meas[2, :], label = "$z_{tip}$",\
            linestyle='-', linewidth=2, markersize=12)
        leg_t = ax_sol_t[1].legend(loc="upper right")
        leg_t.set_draggable(True)
        # ax_sol_t[1].set_xlabel(r"time [s]")
        ax_sol_t[1].set_ylabel(r"hip pos [m]")
        ax_sol_t[1].set_title(r"Tip absolute vertical position", fontdict=None, loc='center')
        ax_sol_t[1].grid()

        f4=plt.figure()
    
        plt.plot(self.sim_replay_time.flatten(), self.sim_tip_pos_meas[0, :], label=r"tip_pos_x", linestyle='-', linewidth=2)
        plt.plot(self.sim_replay_time.flatten(), self.sim_tip_pos_meas[1, :], label=r"tip_pos_y", linestyle='-', linewidth=2)
        plt.plot(self.sim_replay_time.flatten(), self.sim_tip_pos_meas[2, :], label=r"tip_pos_z", linestyle='-', linewidth=2)

        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel('[m]')
        plt.title("Tip absolute position", fontdict=None, loc='center')
        plt.grid()

        f5=plt.figure()

        plt.plot(self.sim_replay_time.flatten(), self.sim_base_link_abs[0, :], label=r"base_link_pos_x", linestyle='-', linewidth=2)
        plt.plot(self.sim_replay_time.flatten(), self.sim_base_link_abs[1, :], label=r"base_link_pos_y", linestyle='-', linewidth=2)
        plt.plot(self.sim_replay_time.flatten(), self.sim_base_link_abs[2, :], label=r"base_link_pos_z", linestyle='-', linewidth=2)

        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel('[m]')
        plt.title("base_link absolute position", fontdict=None, loc='center')
        plt.grid()


    def make_test_plots(self, make_plots = False):
        
        a = False

        if make_plots and self.test_data_loaded_ok:

            a = True

        return a

    def show_plots(self):
        
        plt.show() # show the plots

        return True

    def save_plots(self):

        return True


class LogLoader:

    def __init__(self, mat_file_path):

        self.mat_file_h5py = h5py.File(mat_file_path,'r') # read the .mat file using the H5 .mat Python reader 
        
        self.data = {} # empty dictionary for storing data

        self.mat_file_h5py.visit(self.__h5py_visit_callable) # sliding through the loaded file and processing its fields

    ## Low-level methods for manipulating the .mat file ##

    def __read_str_from_ds(self, ds, index): 

        """

        Reads a string of a string cell array from a h5py database, given the index to be read. Only works with one-dimensional cell arrays of strings.
        
        Args:
            ds: dataset (see h5py docs)
            index: index to be read

        """

        read_str = ""

        try:
            ref = ds[0][index] # list of references to each object in the dataset
            st = self.mat_file_h5py[ref]

            read_str = ''.join(chr(i[0]) for i in st[:])

        except:
            print("ERROR: Could not extract a string from the provided h5py database. \n")

        return read_str

    def __h5py_visit_callable(self, name):

        """

        Callable function passed to the h5py visit() method. 
        Used to perform some useful initializations of the class attributes.

        Args:
            name: field names in the .mat file

        """

        self.data[name] = np.array(self.mat_file_h5py[name]).T   # add the pair key-value to the dictionary

        # if 'plugin_dt' in name: # assigns the aux names and their associated codes to a dictionary

        #     self.data[name] = np.array(self.mat_file_h5py[name])[0][0]   # add the pair key-value to the dictionary
        
        # if 'replay_time' in name: # assigns the aux names and their associated codes to a dictionary

        #     self.data[name] = np.array(self.mat_file_h5py[name]).flatten()   # add the pair key-value to the dictionary
        
        # if 'q_p_meas' in name: # assigns the aux names and their associated codes to a dictionary

        #     self.data[name] = np.array(self.mat_file_h5py[name]).T   # add the pair key-value to the dictionary
        
        # if 'q_p_dot_meas' in name: # assigns the aux names and their associated codes to a dictionary

        #     self.data[name] = np.array(self.mat_file_h5py[name]).T   # add the pair key-value to the dictionary
        
        # if 'q_p_cmd' in name: # assigns the aux names and their associated codes to a dictionary

        #     self.data[name] = np.array(self.mat_file_h5py[name]).T  # add the pair key-value to the dictionary
        
        # if 'q_p_dot_cmd' in name: # assigns the aux names and their associated codes to a dictionary

        #     self.data[name] = np.array(self.mat_file_h5py[name]).T   # add the pair key-value to the dictionary
        
        # if 'replay_damping' in name: # assigns the aux names and their associated codes to a dictionary

        #     self.data[name] = np.array(self.mat_file_h5py[name]).flatten()  # add the pair key-value to the dictionary

        # if 'replay_stiffness' in name: # assigns the aux names and their associated codes to a dictionary

        #     self.data[name] = np.array(self.mat_file_h5py[name]).flatten()  # add the pair key-value to the dictionary
        
        # if 'stop_damping' in name: # assigns the aux names and their associated codes to a dictionary

        #     self.data[name] = np.array(self.mat_file_h5py[name]).flatten()   # add the pair key-value to the dictionary
    
        # if 'stop_stiffness' in name: # assigns the aux names and their associated codes to a dictionary

        #     self.data[name] = np.array(self.mat_file_h5py[name]).flatten()   # add the pair key-value to the dictionary

        # if 'tau_meas' in name: # assigns the aux names and their associated codes to a dictionary

        #     self.data[name] = np.array(self.mat_file_h5py[name]).T   # add the pair key-value to the dictionary

        # if 'tau_cmd' in name: # assigns the aux names and their associated codes to a dictionary

        #     self.data[name] = np.array(self.mat_file_h5py[name]).T   # add the pair key-value to the dictionary

        # if 'tau_ref' in name: # assigns the aux names and their associated codes to a dictionary

        #     self.data[name] = np.array(self.mat_file_h5py[name]).T   # add the pair key-value to the dictionary

        # if 'replay_damping' in name: # assigns the aux names and their associated codes to a dictionary

        #     self.data[name] = np.array(self.mat_file_h5py[name]).flatten()   # add the pair key-value to the dictionar

        # if 'replay_stiffness' in name: # assigns the aux names and their associated codes to a dictionary

        #     self.data[name] = np.array(self.mat_file_h5py[name]).flatten()   # add the pair key-value to the dictionary

        # if 'base_link_abs' in name: # assigns the aux names and their associated codes to a dictionary

        #     self.data[name] = np.array(self.mat_file_h5py[name]).T   # add the pair key-value to the dictionary
        
        # if 'tip_pos_meas' in name: # assigns the aux names and their associated codes to a dictionary

        #     self.data[name] = np.array(self.mat_file_h5py[name]).T  # add the pair key-value to the dictionary
        
        # if 'tip_pos_rel_base_link' in name: # assigns the aux names and their associated codes to a dictionary

        #     self.data[name] = np.array(self.mat_file_h5py[name]).T   # add the pair key-value to the dictionary

        # if 'tip_pos_rel_base_link' in name: # assigns the aux names and their associated codes to a dictionary

        #     self.data[name] = np.array(self.mat_file_h5py[name]).T   # add the pair key-value to the dictionary
        
        # if 'tip_pos_rel_base_link' in name: # assigns the aux names and their associated codes to a dictionary

        #     self.data[name] = np.array(self.mat_file_h5py[name]).T   # add the pair key-value to the dictionary
        
        # if 'tip_pos_rel_base_link' in name: # assigns the aux names and their associated codes to a dictionary

        #     self.data[name] = np.array(self.mat_file_h5py[name]).T   # add the pair key-value to the dictionary

        # if 'tip_pos_rel_base_link' in name: # assigns the aux names and their associated codes to a dictionary

        #     self.data[name] = np.array(self.mat_file_h5py[name]).T   # add the pair key-value to the dictionary
        
        return None # any other value != to None would block the execution of visit() method
