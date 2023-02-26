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

        if 'dropdown_damping' in name: # assigns the aux names and their associated codes to a dictionary

            self.data[name] = np.array(self.mat_file_h5py[name]).flatten()  # add the pair key-value to the dictionary
        
        if 'dropdown_stiffness' in name: # assigns the aux names and their associated codes to a dictionary

            self.data[name] = np.array(self.mat_file_h5py[name]).flatten()  # add the pair key-value to the dictionary

        if 'dropdown_q_ref' in name: # assigns the aux names and their associated codes to a dictionary

            self.data[name] = np.array(self.mat_file_h5py[name]).flatten()  # add the pair key-value to the dictionary

        if 'dropdown_rec_energy' in name: # assigns the aux names and their associated codes to a dictionary

            self.data[name] = np.array(self.mat_file_h5py[name]).flatten()  # add the pair key-value to the dictionary
        
        if 'dropdown_impact_severity_ratio' in name: # assigns the aux names and their associated codes to a dictionary

            self.data[name] = np.array(self.mat_file_h5py[name]).flatten()  # add the pair key-value to the dictionary
     
        return None # any other value != to None would block the execution of visit() method
    
n_samples = 5
n_conf = 6
data_name = "power_data"

# different configurations, fixed stiffness
data_list = []
base_path1 = "/media/andreap/AP_backup/awesome_leg_IROS23_data/awesome_leg_tests/dropdown_tests/sim/const_stifness"
for i in range(0, n_conf):
    data_list.append(LoadDropDownData(base_path1 + "/" + "conf" + str(i) + "/" + data_name + ".mat"))

dropdown_stiff = np.zeros((n_conf, n_samples))
dropdown_damp = np.zeros((n_conf, n_samples))
dropdown_qref = np.zeros((n_conf, n_samples))
dropdown_reg_energy = np.zeros((n_conf, n_samples))
dropdown_sev_ratio = np.zeros((n_conf, n_samples))

for i in range(0, n_conf):
        
    dropdown_stiff[i, :] = data_list[i].data["dropdown_stiffness"][0:n_samples]
    dropdown_damp[i, :] = data_list[i].data["dropdown_damping"][0:n_samples]
    dropdown_qref[i, :] = data_list[i].data["dropdown_q_ref"][0:n_samples]
    dropdown_reg_energy[i, :] = data_list[i].data["dropdown_rec_energy"][0:n_samples] 
    dropdown_sev_ratio[i, :] = data_list[i].data["dropdown_impact_severity_ratio"][0:n_samples]

stiff_setpoint = round(dropdown_stiff[0, 0], 0)
damp_setpoint = round(dropdown_damp[0, 0], 0)

green_diamond = dict(markerfacecolor='g', marker='D')
        
_, ax_sol_t_box = plt.subplots(1)

artist = ax_sol_t_box.boxplot(dropdown_reg_energy.T,
                flierprops = green_diamond, vert=True, 
                whis = 1,
                autorange = True, 
                labels = ["0", "1", "2", "3", "4", "5"], 
                showfliers=False, 
                notch=False)
leg = ax_sol_t_box.legend(artist["boxes"], [rf"stiffness setpoint: {stiff_setpoint} N m/rad",rf"damping setpoint: {damp_setpoint} N m/(rad/s)"], loc='upper right', handlelength=0, handletextpad=0, fancybox=True)           
leg.set_draggable(state = True)
# for item in leg.legendHandles:
#     item.set_visible(False)
ax_sol_t_box.set_ylabel(r"$e_{reg}$ [J]")
ax_sol_t_box.set_xlabel("configuration n.")
ax_sol_t_box.set_title(r"Dropdown tests - fixed stiffness", fontdict=None, loc='center')
ax_sol_t_box.grid()

# same configuration, different stiffnesses
data_list2 = []
n_conf2 = 4
base_path2 = "/media/andreap/AP_backup/awesome_leg_IROS23_data/awesome_leg_tests/dropdown_tests/sim/const_conf"
for i in range(0, n_conf2):
    data_list2.append(LoadDropDownData(base_path2 + "/" + "stiff" + str(i) + "/" + data_name + ".mat"))

dropdown_stiff2 = np.zeros((n_conf2, n_samples))
dropdown_damp2 = np.zeros((n_conf2, n_samples))
dropdown_qref2 = np.zeros((n_conf2, n_samples))
dropdown_reg_energy2 = np.zeros((n_conf2, n_samples))
dropdown_sev_ratio2 = np.zeros((n_conf2, n_samples))

for i in range(0, n_conf2):
        
    dropdown_stiff2[i, :] = data_list2[i].data["dropdown_stiffness"][0:n_samples]
    dropdown_damp2[i, :] = data_list2[i].data["dropdown_damping"][0:n_samples]
    dropdown_qref2[i, :] = data_list2[i].data["dropdown_q_ref"][0:n_samples]
    dropdown_reg_energy2[i, :] = data_list2[i].data["dropdown_rec_energy"][0:n_samples] 
    dropdown_sev_ratio2[i, :] = data_list2[i].data["dropdown_impact_severity_ratio"][0:n_samples]
        
_, ax_sol_t_box2 = plt.subplots(1)
stiff_values = dropdown_stiff2[0:n_conf2, 0]
stiff_labels = [str(int(round(x, 0))) for x in list(stiff_values.tolist())]
ax_sol_t_box2.boxplot(dropdown_reg_energy2.T,
                flierprops = green_diamond, vert=True, 
                whis = 1,
                autorange = True, 
                labels = stiff_labels, 
                showfliers=False, 
                notch=False)
leg2 = ax_sol_t_box2.legend(artist["boxes"],  [rf"configuration number: 4",rf"damping setpoint: {damp_setpoint} N m/(rad/s)"], loc='upper right', handlelength=0, handletextpad=0, fancybox=True)           
leg2.set_draggable(state = True)
ax_sol_t_box2.set_ylabel(r"$e_{reg}$ [J]")
ax_sol_t_box2.set_xlabel("[Nm/rad]")
ax_sol_t_box2.set_title(r"Dropdown tests - fixed landing configuration", fontdict=None, loc='center')
ax_sol_t_box2.grid()

plt.show()
