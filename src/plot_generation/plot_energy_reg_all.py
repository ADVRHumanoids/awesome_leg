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

import os
import fnmatch



class LoadFinalTest:

    def __init__(self, 
                mat_file_path, 
                pattern):
        
        self.mat_file_names = self.get_matching_files(mat_file_path, pattern)

        self.mat_file_h5py = []
        
        self.mat_file_h5py_tmp = None

        self.data = []

        self.data_tmp = {}

        for i in range(0, len(self.mat_file_names)):

            self.mat_file_h5py.append(h5py.File(mat_file_path + "/" + self.mat_file_names[i],'r')) # read the .mat file using the H5 .mat Python reader )

            self.data.append({}) # empty dictionary for storing data

        for i in range(0, len(self.mat_file_names)):
            
            self.data_tmp = {}

            self.mat_file_h5py_tmp = self.mat_file_h5py[i]
            
            self.mat_file_h5py_tmp.visit(self.__h5py_visit_callable) # sliding through the loaded file and processing its fields

            self.data[i] = self.data_tmp

        self.data_keys = list(self.data_tmp.keys())

        self.data_joined = {}

        self.join_all()

    ## Low-level methods for manipulating the .mat file ##
    
    def get_matching_files(self, folder_path, pattern):
        matching_files = []
        for file in os.listdir(folder_path):
            if fnmatch.fnmatch(file, pattern):
                matching_files.append(file)
        return matching_files

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

            self.data_tmp[name] = np.array(self.mat_file_h5py_tmp[name])[0][0]  # add the pair key-value to the dictionary

        # pow stuff
        
        if 'dropdown_impact_severity_ratio' in name: # assigns the aux names and their associated codes to a dictionary

            self.data_tmp[name] = np.array(self.mat_file_h5py_tmp[name])  # add the pair key-value to the dictionary

        if 'dropdown_stiffness' in name: # assigns the aux names and their associated codes to a dictionary

            self.data_tmp[name] = np.array(self.mat_file_h5py_tmp[name]).T  # add the pair key-value to the dictionary

        if 'dropdown_damping' in name: # assigns the aux names and their associated codes to a dictionary

            self.data_tmp[name] = np.array(self.mat_file_h5py_tmp[name]).T  # add the pair key-value to the dictionary

        if 'dropdown_rec_energy_meas' in name: # assigns the aux names and their associated codes to a dictionary

            self.data_tmp[name] = np.array(self.mat_file_h5py_tmp[name]).T  # add the pair key-value to the dictionary
        
        if 'dropdown_rec_energy' in name: # assigns the aux names and their associated codes to a dictionary

            self.data_tmp[name] = np.array(self.mat_file_h5py_tmp[name])  # add the pair key-value to the dictionary

        if 'e_batt' in name: # assigns the aux names and their associated codes to a dictionary

            self.data_tmp[name] = np.array(self.mat_file_h5py_tmp[name])  # add the pair key-value to the dictionary

        if 'ibatt' in name: # assigns the aux names and their associated codes to a dictionary

            self.data_tmp[name] = np.array(self.mat_file_h5py_tmp[name])  # add the pair key-value to the dictionary

        if 'vbatt' in name: # assigns the aux names and their associated codes to a dictionary

            self.data_tmp[name] = np.array(self.mat_file_h5py_tmp[name])  # add the pair key-value to the dictionary

        if 'p_batt' in name: # assigns the aux names and their associated codes to a dictionary

            self.data_tmp[name] = np.array(self.mat_file_h5py_tmp[name])  # add the pair key-value to the dictionary

        if 'est_recov_energy_tot' in name: # assigns the aux names and their associated codes to a dictionary

            self.data_tmp[name] = np.array(self.mat_file_h5py_tmp[name])  # add the pair key-value to the dictionary

        if 'reg_energy' in name: # assigns the aux names and their associated codes to a dictionary

            self.data_tmp[name] = np.array(self.mat_file_h5py_tmp[name])  # add the pair key-value to the dictionary

        # traj replay stuff

        if 'q_p_cmd' in name: # assigns the aux names and their associated codes to a dictionary

            self.data_tmp[name] = np.array(self.mat_file_h5py_tmp[name]).T  # add the pair key-value to the dictionary

        if 'q_p_dot_cmd' in name: # assigns the aux names and their associated codes to a dictionary

            self.data_tmp[name] = np.array(self.mat_file_h5py_tmp[name]).T  # add the pair key-value to the dictionary
        
        if 'tau_cmd' in name: # assigns the aux names and their associated codes to a dictionary

            self.data_tmp[name] = np.array(self.mat_file_h5py_tmp[name]).T  # add the pair key-value to the dictionary
        
        if 'tau_meas' in name: # assigns the aux names and their associated codes to a dictionary

            self.data_tmp[name] = np.array(self.mat_file_h5py_tmp[name]).T  # add the pair key-value to the dictionary

        if 'q_p_meas' in name: # assigns the aux names and their associated codes to a dictionary

            self.data_tmp[name] = np.array(self.mat_file_h5py_tmp[name]).T  # add the pair key-value to the dictionaryù

        if 'q_p_dot_meas' in name: # assigns the aux names and their associated codes to a dictionary

            self.data_tmp[name] = np.array(self.mat_file_h5py_tmp[name]).T  # add the pair key-value to the dictionary
        
        if 'ramp_damping' in name: # assigns the aux names and their associated codes to a dictionary

            self.data_tmp[name] = np.array(self.mat_file_h5py_tmp[name]).T  # add the pair key-value to the dictionary

        if 'ramp_stiffness' in name: # assigns the aux names and their associated codes to a dictionary

            self.data_tmp[name] = np.array(self.mat_file_h5py_tmp[name]).T  # add the pair key-value to the dictionary

        if 'replay_damping' in name: # assigns the aux names and their associated codes to a dictionary

            self.data_tmp[name] = np.array(self.mat_file_h5py_tmp[name]).T  # add the pair key-value to the dictionary
        
        if 'replay_stiffness' in name: # assigns the aux names and their associated codes to a dictionary

            self.data_tmp[name] = np.array(self.mat_file_h5py_tmp[name]).T  # add the pair key-value to the dictionary
        
        if 'iq_ref' in name: # assigns the aux names and their associated codes to a dictionary

            self.data_tmp[name] = np.array(self.mat_file_h5py_tmp[name]).T  # add the pair key-value to the dictionary
        
        if 'f_contact_ref' in name: # assigns the aux names and their associated codes to a dictionary

            self.data_tmp[name] = np.array(self.mat_file_h5py_tmp[name]).T  # add the pair key-value to the dictionary
        
        if 'plugin_time' in name: # assigns the aux names and their associated codes to a dictionary

            self.data_tmp[name] = np.array(self.mat_file_h5py_tmp[name]).T  # add the pair key-value to the dictionary

        return None # any other value != to None would block the execution of visit() method
    
    def join_all(self):

        for name in self.data_keys:

            self.data_joined[name] = self.get_joined(name)

    def get_joined(self, 
            varname: str):
        
        transpose = False

        if (len(self.data[0][varname].shape)) == 0:
            
            n_rows = 1 
            n_cols_base = 1
        
        else:


            if self.data[0][varname].shape[1] == 1 and self.data[0][varname].shape[0] > 1:
                
                n_rows = 1
                n_cols_base = self.data[0][varname].shape[0]
                transpose = True
                
            if not (self.data[0][varname].shape[1] == 1 and self.data[0][varname].shape[0] > 1):
            
                n_rows = self.data[0][varname].shape[0]
                n_cols_base = self.data[0][varname].shape[1]

        n_cols = 0

        if (len(self.data[0][varname].shape)) == 0:
            
            n_cols = 1

        else:

            for i in range(0, len(self.mat_file_names)):
                
                if not transpose:

                    n_cols = n_cols + self.data[i][varname].shape[1]
                
                else:

                    n_cols = n_cols + self.data[i][varname].transpose().shape[1]

        joined_mat = np.zeros((n_rows, n_cols))
        
        index = 0
        for i in range(0, len(self.mat_file_names)):
      
            if not transpose:
                
                if not n_cols == 1:
                    
                    n_cols_local = self.data[i][varname].shape[1]
                    
                else:

                    n_cols_local = 1

                joined_mat[:, index:(index + n_cols_local)] = self.data[i][varname]

            else:
                
                n_cols_local = self.data[i][varname].transpose().shape[1]

                joined_mat[:, index:(index + n_cols_local)] = self.data[i][varname].transpose()

            index = index + n_cols_local

        return joined_mat

path_opt_sim = "/tmp/humanoids_opendata/final_jump_energy_recov/sim/jump_generation_06-07-2023-14_24_01/opt"
path_nonopt_sim = "/tmp/humanoids_opendata/final_jump_energy_recov/sim/jump_generation_06-07-2023-14_24_01/not_opt"
path_opt_hardware = "/tmp/humanoids_opendata/final_jump_energy_recov/real_hardware/jump_generation_06-07-2023-14_24_01/opt"
# path_opt_hardware = "/tmp/dummy"

# path_nonopt_hardware = "/tmp/humanoids_opendata/final_jump_energy_recov/sim/jump_generation_06-07-2023-14_24_01/opt"
data_nameopt= "sim_bus_power_rt__4_2023_07_14__16_28_07"
data_namenonopt0= "trial0/sim_bus_power_rt__4_2023_07_14__15_30_32"
data_namenonopt1= "trial1/sim_bus_power_rt__4_2023_07_14__15_38_15"
data_namenonopt2= "trial2/sim_bus_power_rt__4_2023_07_14__15_44_33"
data_namenonopt3= "trial3/sim_bus_power_rt__4_2023_07_14__15_52_57"
data_namenonopt4= "trial4/sim_bus_power_rt__4_2023_07_14__16_03_42"

# real hardware
# traj_replay_hardware = LoadFinalTest(path_opt_hardware, "test_traj_replay*.mat").data_joined

power_hardare = LoadFinalTest(path_opt_hardware, "test_bus_power*.mat").data_joined

# simulation

data_log_opt = LoadFinalTest(path_opt_sim, "sim_bus_power*.mat").data_joined
data_log_nonopt0 = LoadFinalTest(path_nonopt_sim + "/trial0", "sim_bus_power*.mat").data_joined
data_log_nonopt1 = LoadFinalTest(path_nonopt_sim + "/trial1", "sim_bus_power*.mat").data_joined
data_log_nonopt2 = LoadFinalTest(path_nonopt_sim + "/trial2", "sim_bus_power*.mat").data_joined
data_log_nonopt3 = LoadFinalTest(path_nonopt_sim + "/trial3", "sim_bus_power*.mat").data_joined
data_log_nonopt4 = LoadFinalTest(path_nonopt_sim + "/trial4", "sim_bus_power*.mat").data_joined

f2, ax_sol_t_box2 = plt.subplots(1)
stiff_values_nonopt0 = data_log_nonopt0["dropdown_stiffness"][:, 0]
stiff_values_nonopt1 = data_log_nonopt1["dropdown_stiffness"][:, 0]
stiff_values_nonopt2 = data_log_nonopt2["dropdown_stiffness"][:, 0]
stiff_values_nonopt3 = data_log_nonopt3["dropdown_stiffness"][:, 0]
stiff_values_nonopt4 = data_log_nonopt4["dropdown_stiffness"][:, 0]

damp_values_nonopt0 = data_log_nonopt0["dropdown_damping"][:, 0]
damp_values_nonopt1 = data_log_nonopt1["dropdown_damping"][:, 0]
damp_values_nonopt2 = data_log_nonopt2["dropdown_damping"][:, 0]
damp_values_nonopt3 = data_log_nonopt3["dropdown_damping"][:, 0]
damp_values_nonopt4 = data_log_nonopt4["dropdown_damping"][:, 0]

stiff_values_opt = data_log_opt["dropdown_stiffness"][:, 0]
damp_values_opt = data_log_opt["dropdown_damping"][:, 0]

stiff_values_opt_hardware = power_hardare["dropdown_stiffness"][:, 0]
damp_values_opt_hardware = power_hardare["dropdown_damping"][:, 0]

recov_energy_nonopt0 = data_log_nonopt0["dropdown_rec_energy"][:, :14]
recov_energy_nonopt1 = data_log_nonopt1["dropdown_rec_energy"][:, :14]
recov_energy_nonopt2 = data_log_nonopt2["dropdown_rec_energy"][:, :14]
recov_energy_nonopt3 = data_log_nonopt3["dropdown_rec_energy"][:, :14]
recov_energy_nonopt4 = data_log_nonopt4["dropdown_rec_energy"][:, :14]

recov_energy_opt = data_log_opt["dropdown_rec_energy"][:, :14]

recov_energy_opt_hardware = power_hardare["dropdown_rec_energy_meas"]

combined_data = [recov_energy_nonopt0.flatten().tolist(), 
                recov_energy_nonopt1.flatten().tolist(), 
                recov_energy_nonopt2.flatten().tolist(), 
                recov_energy_nonopt3.flatten().tolist(),
                recov_energy_nonopt4.flatten().tolist(),
                recov_energy_opt.flatten().tolist(), 
                recov_energy_opt_hardware.flatten().tolist()]

labels = [r"${n0}_{\mathrm{sim}}$", 
          r"${n1}_{\mathrm{sim}}$", 
          r"${n2}_{\mathrm{sim}}$", 
          r"${n3}_{\mathrm{sim}}$", 
          r"${n4}_{\mathrm{sim}}$", 
          r"$\mathbf{n5}_{\mathrm{sim}}}$", r"$\mathbf{n5_{\mathrm{real}}}$"]

samples_descriptions = [
                        # "Non opt. " + r"$\hat{q}$" + "; not opt. impedances\n "+  \
                        f"             Kp:" + "                " +  f"   Kd:"+  "\n" + \
                        r"$n0_{\mathrm{sim}}$" + r"$\rightarrow$" + f" [{round(stiff_values_nonopt0[0], 2)}, {round(stiff_values_nonopt0[1], 2)}]; " + f"[{round(damp_values_nonopt0[0], 2)}, {round(damp_values_nonopt0[1], 2)}]",
                        r"$n1_{\mathrm{sim}}$" + r"$\rightarrow$" + f" [{round(stiff_values_nonopt1[0], 2)}, {round(stiff_values_nonopt1[1], 2)}]; " + f"[{round(damp_values_nonopt1[0], 2)}, {round(damp_values_nonopt1[1], 2)}]",
                        r"$n2_{\mathrm{sim}}$" + r"$\rightarrow$" + f" [{round(stiff_values_nonopt2[0], 2)}, {round(stiff_values_nonopt2[1], 2)}]; " + f"[{round(damp_values_nonopt2[0], 2)}, {round(damp_values_nonopt2[1], 2)}]",
                        r"$n3_{\mathrm{sim}}$" + r"$\rightarrow$" + f" [{round(stiff_values_nonopt3[0], 2)}, {round(stiff_values_nonopt3[1], 2)}];     " + f"[{round(damp_values_nonopt3[0], 2)}, {round(damp_values_nonopt3[1], 2)}]",
                        r"$n4_{\mathrm{sim}}$" + r"$\rightarrow$" + f" [{round(stiff_values_nonopt4[0], 2)}, {round(stiff_values_nonopt4[1], 2)}];     " + f"[{round(damp_values_nonopt4[0], 2)}, {round(damp_values_nonopt4[1], 2)}]",
                        r"$\mathbf{n5}_\mathbf{\mathrm{sim}}}$" + r"$\rightarrow$" + f" [{round(stiff_values_opt[0], 2)}, {round(stiff_values_opt[1], 2)}]; "  + f"[{round(damp_values_opt[0], 2)}, {round(damp_values_opt[1], 2)}]",              
                        r"$\mathbf{n5}_\mathbf{\mathrm{real}}$" + r"$\rightarrow$" + f" [{round(stiff_values_opt_hardware[0], 2)}, {round(stiff_values_opt_hardware[1], 2)}]; "  + f"[{round(damp_values_opt_hardware[0], 2)}, {round(damp_values_opt_hardware[1], 2)}]"
                    ]
fontzise = 18
figsizey = 5
figsizex = 10
green_diamond = dict(markerfacecolor='g', marker='D')

artist= ax_sol_t_box2.boxplot(combined_data,
                flierprops = green_diamond, vert=True, 
                whis = 1,
                labels = labels,
                autorange = True, 
                showfliers=False, 
                notch=False)
ax_sol_t_box2.set_xticklabels(labels= labels, rotation = 0, Fontsize=fontzise)

leg2 = ax_sol_t_box2.legend(artist["boxes"],  samples_descriptions, loc='upper right', handlelength=0, handletextpad=0, fancybox=True, fontsize = fontzise)           
leg2.set_draggable(state = True)
ax_sol_t_box2.set_ylabel(r"$e_{\mathrm{batt}}$ [J]", fontsize = fontzise)
ax_sol_t_box2.set_xlabel("", fontsize = fontzise)
ax_sol_t_box2.set_title(r"Post-impact regeneration", fontdict=None, loc='center', fontsize = fontzise)
ax_sol_t_box2.grid()
f2.set_figheight(figsizey)
f2.set_figwidth(figsizex)
plt.show()
