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

import h5py

class LoadDropDownData:

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
        
        if 'p_batt' in name: # assigns the aux names and their associated codes to a dictionary

            self.data[name] = np.array(self.mat_file_h5py[name]).flatten()  # add the pair key-value to the dictionary

        if 'est_pr' in name: # assigns the aux names and their associated codes to a dictionary

            self.data[name] = np.array(self.mat_file_h5py[name]).T  # add the pair key-value to the dictionary

        if 'iq_meas' in name: # assigns the aux names and their associated codes to a dictionary

            self.data[name] = np.array(self.mat_file_h5py[name]).T  # add the pair key-value to the dictionary

        if 'iq_est' in name: # assigns the aux names and their associated codes to a dictionary

            self.data[name] = np.array(self.mat_file_h5py[name]).T # add the pair key-value to the dictionary

        return None # any other value != to None would block the execution of visit() method
    
path = "/tmp/humanoids_opendata/iq_model_tracking_and_reg_pow/reg_pow_tracking/jump_generation_24-02-2023-12_13_45"
data_name= "test_bus_power_rt__0_2023_02_27__21_58_00"

bus_power_leak = - 38 # [W], compensated because missing from logged data
data_log = LoadDropDownData(path + "/" + data_name + ".mat")

n_samples = data_log.data["iq_meas"].shape[1]

time_vect = np.array([data_log.data["plugin_dt"] * i for i in range(0, n_samples)])
t_start = 55.0 # [s]
t_end = 65.0
data_selector = np.array([False ] * n_samples)

for i in range(n_samples):
    data_selector[i] = (time_vect[i] >= t_start and time_vect[i] <= t_end)

iq_meas = data_log.data["iq_meas"]
iq_est = data_log.data["iq_est"]
p_batt = data_log.data["p_batt"]
est_pr = data_log.data["est_pr"]
est_pr_tot = np.sum(est_pr, 0) + bus_power_leak

selected_time = time_vect[data_selector] - time_vect[data_selector][0]
selected_iq_meas = iq_meas[:, data_selector]
selected_iq_est = data_log.data["iq_est"][:, data_selector]
selected_p_meas = data_log.data["p_batt"][data_selector]
selected_p_est = est_pr_tot[data_selector]

fontsize = 14
f1, (ax1, ax2) = plt.subplots(2)
# f1.title()
ax1.plot(selected_time, selected_iq_est[0, :], drawstyle='steps-post')
ax1.plot(selected_time, selected_iq_meas[0, :], drawstyle='steps-post')
# ax1.set_xlabel('time')
ax1.set_ylabel('[A]', fontsize=fontsize)
ax1.set_title('Hip joint', fontsize=fontsize)
ax1.grid()
legend = ax1.legend([r"$i_q$ estimate", r"$i_q$ measurement"], fontsize=fontsize)
legend.set_draggable(state = True)
ax2.plot(selected_time, selected_iq_est[1, :], drawstyle='steps-post')
ax2.plot(selected_time, selected_iq_meas[1, :], drawstyle='steps-post')
ax2.set_xlabel('time [s]', fontsize=fontsize)
ax2.set_ylabel('[A]', fontsize=fontsize)
ax2.set_title('Knee joint', fontsize=fontsize)
ax2.grid()
f1.set_figheight(5)
f1.set_figwidth(10)
# legend = ax2.legend([r"$i_q$ estimate", r"$i_q$ measurement"])
# legend.set_draggable(state = True)

f, ax = plt.subplots(1)
# f1.title()
ax.plot(selected_time, selected_p_est, drawstyle='steps-post')
ax.plot(selected_time, selected_p_meas, drawstyle='steps-post')
ax.set_xlabel('time [s]', fontsize=fontsize)
ax.set_ylabel('$p_{\mathrm{batt}}$ [W]', fontsize=fontsize)
ax.grid()
ax.set_title('Regenerative power model validation', fontsize=fontsize)
legend = ax.legend([r"estimate", r"measurement"], fontsize=fontsize)
legend.set_draggable(state = True)
f.set_figheight(3)
f.set_figwidth(10)
plt.show()
