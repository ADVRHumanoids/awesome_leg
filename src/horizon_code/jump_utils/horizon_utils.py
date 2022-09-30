from horizon.utils import kin_dyn
import numpy as np

from horizon.utils import mat_storer

import matplotlib.pyplot as plt
# from matplotlib import cm
# from matplotlib.ticker import LinearLocator, FormatStrFormatter

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
                solution_fullpath: str, 
                save_fullpath = "/tmp", 
                save_plots = False):

        self.solution_fullpath = solution_fullpath

        self.__load_sol()

        self.__read_sol()

        self.__compute_time_vect()

        
    def __load_sol(self):
        
        self.ms_loader = mat_storer.matStorer(self.solution_fullpath)
        self.solution=self.ms_loader.load() # loading the solution dictionary

    def __read_sol(self):

        self.q_p=self.solution["q_p"] # excluding test rig dof
        self.q_p_dot=self.solution["q_p_dot"]
        self.q_p_ddot=self.solution["q_p_ddot"]
        self.i_q=self.solution["i_q"]
        self.GRF=self.solution["f_contact"]
        self.tau=self.solution["tau"][1:3,:] 
        self.dt=self.solution["dt_opt"].flatten()
        self.hip_height=self.solution["hip_height"]
        self.foot_tip_height=self.solution["foot_tip_height"]
        self.foot_tip_vel=self.solution["tip_velocity"]
        self.hip_vel=self.solution["hip_velocity"]

    def __compute_time_vect(self):

        self.time_vector = np.zeros([self.tau[0,:].size+1])
        for i in range(self.tau[0,:].size):
            self.time_vector[i+1] = self.time_vector[i] + self.dt[i]

    def make_plots(self):

        f1=plt.figure()
        plt.plot(self.time_vector[1:-1], self.GRF[0, :-1], label=r"F_x")
        plt.plot(self.time_vector[1:-1], self.GRF[1, :-1], label=r"F_y")
        plt.plot(self.time_vector[1:-1], self.GRF[2, :-1], label=r"F_z")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel('[N]')
        plt.title("Ground reaction forces", fontdict=None, loc='center')
        plt.grid()
        # if save_fig:
        #     plt.savefig(save_path+"GRF.pdf", format="pdf")

        f2=plt.figure()
        plt.plot(self.time_vector[:-1], self.q_p[0, :-1], label=r"$\mathrm{q}_{\mathrm{hip}}$")
        plt.plot(self.time_vector[:-1], self.q_p[1, :-1], label=r"$\mathrm{q}_{\mathrm{knee}}$")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[rad]")
        plt.title("Joint positions", fontdict=None, loc='center')
        plt.grid()

        f3=plt.figure()
        plt.plot(self.time_vector[:-1], self.q_p_dot[0, :-1], label=r"$\dot{q}_{\mathrm{hip}}$")
        plt.plot(self.time_vector[:-1], self.q_p_dot[1, :-1], label=r"$\dot{q}_{\mathrm{knee}}$")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[rad/s]")
        plt.title("Joint velocities", fontdict=None, loc='center')
        plt.grid()

        f4=plt.figure()
        plt.plot(self.time_vector[:-1], self.q_p_ddot[0, :], label=r"$\ddot{\mathrm{q}}_{\mathrm{hip}}$")
        plt.plot(self.time_vector[:-1], self.q_p_ddot[1, :], label=r"$\ddot{\mathrm{q}}_{\mathrm{knee}}$")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[$\mathrm{rad}/\mathrm{s}^2$]")
        plt.title("Joint accelerations", fontdict=None, loc='center')
        plt.grid()
    
        f5=plt.figure()
        plt.plot(self.time_vector[:-1], self.tau[0, :],  drawstyle='steps-post', label=r"$\tau_{\mathrm{hip}}$ torque")
        plt.plot(self.time_vector[:-1], self.tau[1, :],  drawstyle='steps-post', label=r"$\tau_{\mathrm{knee}}$ torque")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[Nm]")
        plt.title("Joint efforts (link side)", fontdict=None, loc='center')
        plt.grid()
        
        f6=plt.figure()
        plt.plot(self.time_vector[:-1], self.i_q[0, :], label=r"$\mathrm{i}_\mathrm{q}^{\mathrm{hip}}$ current")
        plt.plot(self.time_vector[:-1], self.i_q[1, :], label=r"$\mathrm{i}_\mathrm{q}^{\mathrm{knee}}$ current")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[A]")
        plt.title("Actuators quadrature current", fontdict=None, loc='center')
        plt.grid()
    
        f8=plt.figure()
        plt.plot(self.time_vector[:-1], self.dt, drawstyle='steps-post')
        plt.legend(loc="upper left")
        plt.ylabel(r"t [s]")
        plt.xlabel("node number")
        plt.title("Optimization dt", fontdict=None, loc='center')
        plt.grid()

        f10=plt.figure()
        plt.plot(self.time_vector, self.hip_height,label="hip height")
        plt.plot(self.time_vector, self.foot_tip_height,label="foot tip height")
        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[m]")
        plt.title("Link heights during the jump", fontdict=None, loc='center')
        plt.grid()
        
        f11=plt.figure()
        plt.plot(self.time_vector, self.foot_tip_vel[0,:],label="tip vel. x")
        plt.plot(self.time_vector, self.foot_tip_vel[1,:],label="tip vel. y")
        plt.plot(self.time_vector, self.foot_tip_vel[2,:],label="tip vel. z")

        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[m/s]")
        plt.title("Foot tip velocity (cartesian components)", fontdict=None, loc='center')
        plt.grid()
        
        f12=plt.figure()
        plt.plot(self.time_vector, self.hip_vel[0,:],label="hip vel. x")
        plt.plot(self.time_vector, self.hip_vel[1,:],label="hip vel. y")
        plt.plot(self.time_vector, self.hip_vel[2,:],label="hip vel. z")

        plt.legend(loc="upper left")
        plt.xlabel(r"t [s]")
        plt.ylabel(r"[m/s]")
        plt.title("Hip velocity (cartesian components)", fontdict=None, loc='center')
        plt.grid()

        return True

    def show_plots(self):
        
        plt.show() # show the plots

        return True

    def save_plots(self):

        return True
