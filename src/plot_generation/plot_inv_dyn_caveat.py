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
from horizon.utils import mat_storer

class LoadFinalTest:

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
        
        if 'tau' in name: # assigns the aux names and their associated codes to a dictionary

            self.data[name] = np.array(self.mat_file_h5py[name]).T  # add the pair key-value to the dictionary

        if 'q_p_ddot' in name: # assigns the aux names and their associated codes to a dictionary

            self.data[name] = np.array(self.mat_file_h5py[name])  # add the pair key-value to the dictionary

        if 'f_contact' in name: # assigns the aux names and their associated codes to a dictionary

            self.data[name] = np.array(self.mat_file_h5py[name]).T  # add the pair key-value to the dictionary

        return None # any other value != to None would block the execution of visit() method
    
path = "/tmp/humanoids_opendata/refinement_motivation/jump_generation_24-02-2023-11_00_02"
data= "apex_awesome_jump_ref"
ms_loader_ref = mat_storer.matStorer(path + "/" + data + ".mat")
opt_data=ms_loader_ref.load() 
# data_log_opt = LoadFinalTest(path + "/" + data + ".mat")
n_samples = opt_data["tau"].shape[1]
time_vect = np.array([0.001 * i for i in range(0, n_samples)])

fontsize = 14
f, ax = plt.subplots(2)

ax[0].plot(time_vect, opt_data["q_p_ddot"][0,:], drawstyle='steps-post')
ax[0].plot(time_vect, opt_data["q_p_ddot"][1,:], drawstyle='steps-post')
ax[0].plot(time_vect, opt_data["q_p_ddot"][2,:], drawstyle='steps-post')

# ax[0].set_xlabel('time',  fontsize=fontsize)
ax[0].set_ylabel(r'[$\mathrm{rad/s^2}$]', fontsize=fontsize)
ax[0].set_title(r"$\dot{v}$", fontsize=fontsize)
ax[0].grid()
legend = ax[0].legend([r"$\dot{v}_{0}$", "$\dot{v}_{1}$", r"$\dot{v}_{2}$"], fontsize=fontsize)
legend.set_draggable(state = True)

ax[1].plot(time_vect, opt_data["f_contact"][2,:], drawstyle='steps-post')
# ax1.plot(selected_time_replay, selected_iq_meas[0, :], drawstyle='steps-post')
ax[1].set_xlabel('time [s]',  fontsize=fontsize)
ax[1].set_ylabel('[N]', fontsize=fontsize)
ax[1].set_title(f'$f$', fontsize=fontsize)
ax[1].grid()
legend = ax[1].legend([r"$f_n$"], fontsize=fontsize)
legend.set_draggable(state = True)
f.set_figheight(5)
f.set_figwidth(10)
plt.show()

plt.show()
