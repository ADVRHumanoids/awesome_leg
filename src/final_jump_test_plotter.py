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
        
        if 'dropdown_impact_severity_ratio' in name: # assigns the aux names and their associated codes to a dictionary

            self.data[name] = np.array(self.mat_file_h5py[name])  # add the pair key-value to the dictionary

        if 'dropdown_rec_energy' in name: # assigns the aux names and their associated codes to a dictionary

            self.data[name] = np.array(self.mat_file_h5py[name])  # add the pair key-value to the dictionary

        if 'dropdown_stiffness' in name: # assigns the aux names and their associated codes to a dictionary

            self.data[name] = np.array(self.mat_file_h5py[name]).T  # add the pair key-value to the dictionary

        if 'dropdown_damping' in name: # assigns the aux names and their associated codes to a dictionary

            self.data[name] = np.array(self.mat_file_h5py[name]).T  # add the pair key-value to the dictionary

        return None # any other value != to None would block the execution of visit() method
    
path_opt = "/media/andreap/AP_backup/awesome_leg_IROS23_data/awesome_leg_tests/final_jump_energy_recov/sim/jump_generation_24-02-2023-12_13_45/opt"
path_nonopt = "/media/andreap/AP_backup/awesome_leg_IROS23_data/awesome_leg_tests/final_jump_energy_recov/sim/jump_generation_24-02-2023-12_13_45/not_opt"
data_nameopt= "sim_bus_power_rt__4_2023_03_02__18_06_34"
data_namenonopt= "sim_bus_power_rt__4_2023_03_01__18_51_41"

data_log_opt = LoadFinalTest(path_opt + "/" + data_nameopt + ".mat")
data_log_nonopt = LoadFinalTest(path_nonopt + "/" + data_namenonopt + ".mat")

f2, ax_sol_t_box2 = plt.subplots(1)
stiff_values_nonopt = data_log_nonopt.data["dropdown_stiffness"][:, 0]
damp_values_nonopt = data_log_nonopt.data["dropdown_damping"][:, 0]
stiff_values_opt = data_log_opt.data["dropdown_stiffness"][:, 0]
damp_values_opt = data_log_opt.data["dropdown_damping"][:, 0]
# print(data_log_opt.data["dropdown_damping"])
# exit()
recov_energy_nonopt = np.array(data_log_nonopt.data["dropdown_rec_energy"])[:-1]
recov_energy_opt = np.array(data_log_opt.data["dropdown_rec_energy"])
combined_data = np.concatenate((recov_energy_nonopt, recov_energy_opt), axis = 1)

impedance_setpoints_opt = [f"non opt.: [{round(stiff_values_nonopt[0], 0)}, {round(stiff_values_nonopt[1], 0)}] N m/rad, " + f"[{round(damp_values_nonopt[0], 0)}, {round(damp_values_nonopt[1], 0)}] N m/(rad/s)",
                           f"opt: [{round(stiff_values_opt[0], 0)}, {round(stiff_values_opt[1], 0)}] N m/rad, " + f"[{round(damp_values_opt[0], 0)}, {round(damp_values_opt[1], 0)}] N m/(rad/s)" 
                            ]

fontzise = 14
figsizey = 5
figsizex = 10
green_diamond = dict(markerfacecolor='g', marker='D')

artist= ax_sol_t_box2.boxplot(combined_data,
                flierprops = green_diamond, vert=True, 
                whis = 1,
                labels = ["non opt.", "opt."],
                autorange = True, 
                showfliers=False, 
                notch=False)
leg2 = ax_sol_t_box2.legend(artist["boxes"],  impedance_setpoints_opt, loc='upper right', handlelength=0, handletextpad=0, fancybox=True, fontsize = fontzise)           
leg2.set_draggable(state = True)
ax_sol_t_box2.set_ylabel(r"$e_{reg}$ [J]", fontsize = fontzise)
ax_sol_t_box2.set_xlabel("", fontsize = fontzise)
ax_sol_t_box2.set_title(r"Post-impact regeneration", fontdict=None, loc='center', fontsize = fontzise)
ax_sol_t_box2.grid()
f2.set_figheight(figsizey)
f2.set_figwidth(figsizex)
plt.show()
