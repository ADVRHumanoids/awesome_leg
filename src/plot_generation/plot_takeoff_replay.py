#!/usr/bin/env python3

from xbot_interface import xbot_interface as xbot
from xbot_interface import config_options as opt

import rospkg

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm
from matplotlib.colors import LightSource, Normalize
from mpl_toolkits.axes_grid1 import make_axes_locatable
from horizon.utils import mat_storer

import h5py

class LoadJumpTrial:

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

        if 'plugin_dt' in name: # assigns the aux names and their associated codes to a dictionary

            self.data[name] = np.array(self.mat_file_h5py[name])[0][0]  # add the pair key-value to the dictionary
        
        if 'est_recov_energy_tot' in name: # assigns the aux names and their associated codes to a dictionary

            self.data[name] = np.array(self.mat_file_h5py[name]).flatten()  # add the pair key-value to the dictionary

        if 'reg_energy' in name: # assigns the aux names and their associated codes to a dictionary

            self.data[name] = np.array(self.mat_file_h5py[name]).flatten()  # add the pair key-value to the dictionary

        if 'q_p_dot_meas' in name: # assigns the aux names and their associated codes to a dictionary

            self.data[name] = np.array(self.mat_file_h5py[name]).T  # add the pair key-value to the dictionary
        
        if 'q_p_dot_cmd' in name: # assigns the aux names and their associated codes to a dictionary

            self.data[name] = np.array(self.mat_file_h5py[name]).T  # add the pair key-value to the dictionary

        if 'q_p_dot_cmd' in name: # assigns the aux names and their associated codes to a dictionary

            self.data[name] = np.array(self.mat_file_h5py[name]).T  # add the pair key-value to the dictionary

        if 'tau_meas' in name: # assigns the aux names and their associated codes to a dictionary

            self.data[name] = np.array(self.mat_file_h5py[name]).T  # add the pair key-value to the dictionary

        if 'iq_meas' in name: # assigns the aux names and their associated codes to a dictionary

            self.data[name] = np.array(self.mat_file_h5py[name]).T  # add the pair key-value to the dictionary

        if 'iq_est' in name: # assigns the aux names and their associated codes to a dictionary

            self.data[name] = np.array(self.mat_file_h5py[name]).T  # add the pair key-value to the dictionary

        return None # any other value != to None would block the execution of visit() method

class LoadOptJump:

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

        if 'dt_opt' in name: # assigns the aux names and their associated codes to a dictionary

            self.data[name] = np.array(self.mat_file_h5py[name])[0][0]  # add the pair key-value to the dictionary

        if 'q_p_dot' in name: # assigns the aux names and their associated codes to a dictionary

            self.data[name] = np.array(self.mat_file_h5py[name]).T  # add the pair key-value to the dictionary

        if 'tau' in name: # assigns the aux names and their associated codes to a dictionary

            self.data[name] = np.array(self.mat_file_h5py[name]).T  # add the pair key-value to the dictionary

        if 'i_q' in name: # assigns the aux names and their associated codes to a dictionary

            self.data[name] = np.array(self.mat_file_h5py[name]).T  # add the pair key-value to the dictionary

        return None # any other value != to None would block the execution of visit() method
    
path = "/tmp/humanoids_opendata/final_jump_energy_recov/real_hardware/backup_jump_tests/jump_generation_06-07-2023-14_24_01"
power_data = "test_bus_power_rt__4_2023_02_28__14_47_58"
replay_data = "test_traj_replay__2_2023_02_28__14_47_58"
mat_name = "apex_awesome_jump_ref"

power_data_log = LoadJumpTrial(path + "/" + power_data + ".mat")
replay_data_log = LoadJumpTrial(path + "/" + replay_data + ".mat")
ms_loader_ref = mat_storer.matStorer(path + "/" + mat_name + ".mat")
opt_data=ms_loader_ref.load() 

n_samples_pow = power_data_log.data["iq_meas"].shape[1]
n_samples_replay = replay_data_log.data["q_p_dot_meas"].shape[1]
n_samples_opt = opt_data["tau"].shape[1]

time_vect_replay = np.array([replay_data_log.data["plugin_dt"] * i for i in range(0, n_samples_replay)])
time_vect_pow = np.array([power_data_log.data["plugin_dt"] * i for i in range(0, n_samples_pow)])
time_vect_opt = np.array([0.001 * i for i in range(0, n_samples_opt)])

t_start = 15.5 # [s]
t_end = 16.4

if (t_end > time_vect_replay[-1]):
    t_end = time_vect_replay[-1]

if (t_end > time_vect_pow[-1]):
    t_end = time_vect_pow[-1]

data_selector_pow = np.array([False] * n_samples_pow)
data_selector_replay = np.array([False] * n_samples_replay)

for i in range(n_samples_pow):
    data_selector_pow[i] = (time_vect_pow[i] >= t_start and time_vect_pow[i] <= t_end)
for i in range(n_samples_replay):
    data_selector_replay[i] = (time_vect_replay[i] >= t_start and time_vect_replay[i] <= t_end)

# print(power_data_log)
# exit()
iq_meas = power_data_log.data["iq_meas"]
iq_est = power_data_log.data["iq_est"]
q_dot_meas = replay_data_log.data["q_p_dot_cmd"]
tau_meas_pow_log = power_data_log.data["tau_meas"]
tau_meas = replay_data_log.data["tau_meas"]
est_e_rec = power_data_log.data["est_recov_energy_tot"]
meas_e_rec = power_data_log.data["reg_energy"]

selected_time_replay = time_vect_replay[data_selector_replay] - time_vect_replay[data_selector_replay][0]
selected_time_pow = time_vect_pow[data_selector_pow] - time_vect_pow[data_selector_pow][0]

selected_iq_est = iq_est[:, data_selector_pow]
selected_iq_meas = iq_meas[:, data_selector_pow]
selected_q_dot_meas = q_dot_meas[:, data_selector_replay]
selected_tau_meas = tau_meas[:, data_selector_replay]
selected_tau_meas_pow = tau_meas_pow_log[:, data_selector_pow]
selected_est_e_rec = est_e_rec[ data_selector_pow]
selected_meas_e_rec = meas_e_rec[data_selector_pow]

## Quadrature current - joint efforts -> takeoff replay on real robot ##

fontsize = 14
f1, ax = plt.subplots(2)
# f1.title()
# ax[0].plot(selected_time_replay, selected_q_dot_meas[0, :])
# ax[0].plot(selected_time_replay, selected_q_dot_meas[1, :])
# ax[0].set_ylabel('[rad/s]', fontsize=fontsize)
# ax[0].set_title('Joint velocities - hardware test', fontsize=fontsize)
# ax[0].grid()
# legend = ax[0].legend([r"$v_{1}$", r"$v_{2}$"], fontsize=fontsize)
# legend.set_draggable(state = True)

ax[0].plot(selected_time_pow, selected_iq_est[0, :])
ax[0].plot(selected_time_pow, selected_iq_meas[1, :])
ax[0].set_ylabel('[A]', fontsize=fontsize)
ax[0].set_title('Quadrature currents - hardware test', fontsize=fontsize)
ax[0].grid()
legend = ax[0].legend([r"$i_{q0}$", r"$i_{q1}$"], fontsize=fontsize)
legend.set_draggable(state = True)

ax[1].plot(selected_time_replay, selected_tau_meas[0, :])
ax[1].plot(selected_time_replay, selected_tau_meas[1, :])
ax[1].set_xlabel('time [s]',  fontsize=fontsize)
ax[1].set_ylabel(r'\tau [Nm]', fontsize=fontsize)
ax[1].set_title('Joint efforts - hardware test', fontsize=fontsize)
ax[1].grid()
legend = ax[1].legend([r"$\tau_{1}$", r"$\tau_{2}$"], fontsize=fontsize)
legend.set_draggable(state = True)

f1.set_figheight(5)
f1.set_figwidth(10)

## Optimized trajectory replay on real robot plots ##

fontsize = 14
f2, ax2 = plt.subplots(2)

ax2[0].plot(selected_time_pow, selected_iq_meas[0, :])
ax2[0].plot(selected_time_pow, selected_iq_est[0, :])
ax2[0].set_xlabel('time [t]', fontsize=fontsize)
ax2[0].set_ylabel(r'$i_q$ [A]', fontsize=fontsize)
ax2[0].set_title(r'$i_q$ model tracking - hip', fontsize=fontsize)
ax2[0].grid()
legend = ax2[0].legend(["estimated" , "meas."], fontsize=fontsize)
legend.set_draggable(state = True)

combined_data = np.stack((selected_iq_meas[0, :], 
                        selected_iq_meas[1, :], 
                        selected_tau_meas_pow[0, :], 
                        selected_tau_meas_pow[1, :]), axis = 1)

green_diamond = dict(markerfacecolor='g', marker='D')
artist= ax2[1].boxplot(combined_data,
            flierprops = green_diamond, vert=True, 
            whis = 1,
            labels = [r"$i_q$" + " hip", r"$i_q$" + " knee", r"$\tau$" + " hip", r"$\tau$" + " knee"],
            autorange = True, 
            showfliers=True, 
            notch=False)
# leg2 = ax2[1].legend(artist["boxes"],  [], loc='upper right', handlelength=0, handletextpad=0, fancybox=True, fontsize = fontsize)           
# leg2.set_draggable(state = True)
ax2[1].set_ylabel(r"$i_q$ [A]" + " - " +  r"$\tau$ [Nm]", fontsize = fontsize)
ax2[1].set_xlabel("", fontsize = fontsize)
ax2[1].set_title(r"Take-off $i_q$ and $\tau$", fontdict=None, loc='center', fontsize = fontsize)
ax2[1].grid()

f2.set_figheight(5)
f2.set_figwidth(10)

## Optimized trajectory data ##

f32, ax32 = plt.subplots(5)
# f1.title()
ax32[0].plot(time_vect_opt, opt_data["q_p_dot"][0, :-1])
ax32[0].plot(time_vect_opt, opt_data["q_p_dot"][1, :-1])
ax32[0].set_ylabel('[rad/s]', fontsize=fontsize)
ax32[0].set_title('Joint velocities - opt solution', fontsize=fontsize)
ax32[0].grid()
legend = ax32[0].legend([r"$v_{1}$", r"$v_{2}$"], fontsize=fontsize)
legend.set_draggable(state = True)

ax32[1].plot(time_vect_opt, opt_data["i_q"][0, :])
ax32[1].plot(time_vect_opt, opt_data["i_q"][1, :])
ax32[1].set_ylabel('[A]', fontsize=fontsize)
ax32[1].set_title('Quadrature currents - opt solution', fontsize=fontsize)
ax32[1].grid()
legend = ax32[1].legend([r"$i_{q0}$", r"$i_{q1}$"], fontsize=fontsize)
legend.set_draggable(state = True)
ax32[2].plot(time_vect_opt, opt_data["tau"][1, :])
ax32[2].plot(time_vect_opt, opt_data["tau"][2, :])
# ax1.plot(selected_time_replay, selected_iq_meas[0, :])
ax32[2].set_ylabel('[Nm]', fontsize=fontsize)
ax32[2].set_title('Joint efforts - opt solution', fontsize=fontsize)
ax32[2].grid()
legend = ax32[2].legend([r"$\tau_{1}$", r"$\tau_{2}$"], fontsize=fontsize)
legend.set_draggable(state = True)

ax32[3].plot(time_vect_opt, opt_data["foot_tip_height"][:-1])
ax32[3].plot(time_vect_opt, opt_data["com_pos"][2, :-1])
ax32[3].set_ylabel('[m]', fontsize=fontsize)
ax32[3].set_title('tip/CoM height - opt solution', fontsize=fontsize)
ax32[3].grid()
legend = ax32[3].legend([f"tip", f"CoM"], fontsize=fontsize)
legend.set_draggable(state = True)

ax32[4].plot(time_vect_opt, opt_data["f_contact"][0, :])
ax32[4].plot(time_vect_opt, opt_data["f_contact"][1, :])
ax32[4].plot(time_vect_opt, opt_data["f_contact"][2, :])
ax32[4].set_xlabel('time [s]',  fontsize=fontsize)
ax32[4].set_ylabel('[N]', fontsize=fontsize)
ax32[4].set_title('contact force - opt solution', fontsize=fontsize)
ax32[4].grid()
legend = ax32[4].legend([r"$f_0$", r"$f_1$", r"$f_2$"], fontsize=fontsize)
legend.set_draggable(state = True)

f32.set_figheight(8)
f32.set_figwidth(13)


plt.show()
