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
    
path_opt = "/tmp/humanoids_opendata/final_jump_energy_recov/sim/jump_generation_06-07-2023-14_24_01/opt"
path_nonopt = "/tmp/humanoids_opendata/final_jump_energy_recov/sim/jump_generation_06-07-2023-14_24_01/not_opt"
data_nameopt= "sim_bus_power_rt__4_2023_07_11__14_33_15"
data_namenonopt0= "trial0/sim_bus_power_rt__4_2023_07_11__13_50_26"
data_namenonopt1= "trial1/sim_bus_power_rt__4_2023_07_11__14_00_28"
data_namenonopt2= "trial2/sim_bus_power_rt__4_2023_07_11__14_10_10"
data_namenonopt3= "trial3/sim_bus_power_rt__4_2023_07_11__14_22_10"
data_namenonopt4= "trial4/sim_bus_power_rt__4_2023_07_11__14_47_51"

data_log_opt = LoadFinalTest(path_opt + "/" + data_nameopt + ".mat")
data_log_nonopt0 = LoadFinalTest(path_nonopt + "/" + data_namenonopt0 + ".mat")
data_log_nonopt1 = LoadFinalTest(path_nonopt + "/" + data_namenonopt1 + ".mat")
data_log_nonopt2 = LoadFinalTest(path_nonopt + "/" + data_namenonopt2 + ".mat")
data_log_nonopt3 = LoadFinalTest(path_nonopt + "/" + data_namenonopt3 + ".mat")
data_log_nonopt4 = LoadFinalTest(path_nonopt + "/" + data_namenonopt4 + ".mat")

f2, ax_sol_t_box2 = plt.subplots(1)
stiff_values_nonopt0 = data_log_nonopt0.data["dropdown_stiffness"][:, 0]
stiff_values_nonopt1 = data_log_nonopt1.data["dropdown_stiffness"][:, 0]
stiff_values_nonopt2 = data_log_nonopt2.data["dropdown_stiffness"][:, 0]
stiff_values_nonopt3 = data_log_nonopt3.data["dropdown_stiffness"][:, 0]
stiff_values_nonopt4 = data_log_nonopt4.data["dropdown_stiffness"][:, 0]

damp_values_nonopt0 = data_log_nonopt0.data["dropdown_damping"][:, 0]
damp_values_nonopt1 = data_log_nonopt1.data["dropdown_damping"][:, 0]
damp_values_nonopt2 = data_log_nonopt2.data["dropdown_damping"][:, 0]
damp_values_nonopt3 = data_log_nonopt3.data["dropdown_damping"][:, 0]
damp_values_nonopt4 = data_log_nonopt4.data["dropdown_damping"][:, 0]

stiff_values_opt = data_log_opt.data["dropdown_stiffness"][:, 0]
damp_values_opt = data_log_opt.data["dropdown_damping"][:, 0]
# print(data_log_opt.data["dropdown_damping"])
# exit()
recov_energy_nonopt0 = np.array(data_log_nonopt0.data["dropdown_rec_energy"])
recov_energy_nonopt1 = np.array(data_log_nonopt1.data["dropdown_rec_energy"])
recov_energy_nonopt2 = np.array(data_log_nonopt2.data["dropdown_rec_energy"])[:-2]
recov_energy_nonopt3 = np.array(data_log_nonopt3.data["dropdown_rec_energy"])
recov_energy_nonopt4 = np.array(data_log_nonopt4.data["dropdown_rec_energy"])

recov_energy_opt = np.array(data_log_opt.data["dropdown_rec_energy"])
combined_data = np.concatenate((recov_energy_nonopt0, 
                            recov_energy_nonopt1, 
                            recov_energy_nonopt2, 
                            recov_energy_nonopt3,
                            recov_energy_nonopt4,
                            recov_energy_opt), axis = 1)

samples_descriptions = [
                        # "Non opt. " + r"$\hat{q}$" + "; not opt. impedances\n "+  \
                        f"                  Kp" + "              " +  f"    Kd"+  "\n" + \
                        f"n0" + r"$\rightarrow$" + f" [{round(stiff_values_nonopt0[0], 2)}, {round(stiff_values_nonopt0[1], 2)}]; " + f"[{round(damp_values_nonopt0[0], 2)}, {round(damp_values_nonopt0[1], 2)}]",
                        f"n1" + r"$\rightarrow$" + f" [{round(stiff_values_nonopt1[0], 2)}, {round(stiff_values_nonopt1[1], 2)}]; " + f"[{round(damp_values_nonopt1[0], 2)}, {round(damp_values_nonopt1[1], 2)}]",
                        f"n2" + r"$\rightarrow$" + f" [{round(stiff_values_nonopt2[0], 2)}, {round(stiff_values_nonopt2[1], 2)}]; " + f"[{round(damp_values_nonopt2[0], 2)}, {round(damp_values_nonopt2[1], 2)}]",
                        f"n3" + r"$\rightarrow$" + f" [{round(stiff_values_nonopt3[0], 2)}, {round(stiff_values_nonopt3[1], 2)}];     " + f"[{round(damp_values_nonopt3[0], 2)}, {round(damp_values_nonopt3[1], 2)}]",
                        f"n4" + r"$\rightarrow$" + f" [{round(stiff_values_nonopt4[0], 2)}, {round(stiff_values_nonopt4[1], 2)}];     " + f"[{round(damp_values_nonopt4[0], 2)}, {round(damp_values_nonopt4[1], 2)}]",
                        r"$\mathbf{n5}$" + r"$\rightarrow$" + f" [{round(stiff_values_opt[0], 2)}, {round(stiff_values_opt[1], 2)}];     "  + f"[{round(damp_values_opt[0], 2)}, {round(damp_values_opt[1], 2)}]",              
                        ]
fontzise = 18
figsizey = 5
figsizex = 10
green_diamond = dict(markerfacecolor='g', marker='D')

artist= ax_sol_t_box2.boxplot(combined_data,
                flierprops = green_diamond, vert=True, 
                whis = 1,
                labels = ["n0", "n1", "n.2", "n.3", "n.4", r"$\mathbf{n5}$"],
                autorange = True, 
                showfliers=False, 
                notch=False)

leg2 = ax_sol_t_box2.legend(artist["boxes"],  samples_descriptions, loc='upper right', handlelength=0, handletextpad=0, fancybox=True, fontsize = fontzise)           
leg2.set_draggable(state = True)
ax_sol_t_box2.set_ylabel(r"$e_{reg}$ [J]", fontsize = fontzise)
ax_sol_t_box2.set_xlabel("", fontsize = fontzise)
ax_sol_t_box2.set_title(r"Post-impact regeneration", fontdict=None, loc='center', fontsize = fontzise)
ax_sol_t_box2.grid()
f2.set_figheight(figsizey)
f2.set_figwidth(figsizex)
plt.show()
