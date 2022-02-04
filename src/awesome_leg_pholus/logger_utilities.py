#!/usr/bin/env python3

import h5py # for loading H5 .mat files

import rospkg

import numpy as np

import matplotlib.pyplot as plt
from matplotlib.cm import get_cmap
from matplotlib.lines import Line2D

################################### LOG LOADER class ###################################

class LogLoader:

    def __init__(self, mat_file_path):
        
        """
        Class constructor.

        Args:
            mat_file_path: absolute path to the .mat file to be loaded

        """

        self.mat_file_h5py = h5py.File(mat_file_path,'r') # read the .mat file using the H5 .mat Python reader 
        
        self.aux_signals = dict() # empty dictionary for storing the codes of the aux signal, as read from the loaded .mat file
        self.joints = dict() # empty dictionary for storing joint codes
        self.chains = dict() # empty dictionary for storing chain names

        self.mat_file_h5py.visit(self.__h5py_visit_callable) # sliding through the loaded file and processing its fields

        # aux message information
        self.aux_seq = np.array(self.mat_file_h5py.get('aux_seq')).astype(int).flatten()
        self.aux_abs_time = np.array(self.mat_file_h5py.get('aux_time')).flatten()

        self.aux_type = np.array(self.mat_file_h5py.get('aux_type')).astype(int).transpose()

        self.aux_values = np.array(self.mat_file_h5py.get('aux_value')).transpose()
        
        # "standard" message information
        self.seq = np.array(self.mat_file_h5py.get('seq')).astype(int).flatten().transpose()
        self.abs_time = np.array(self.mat_file_h5py.get('time')).flatten().transpose()
        
        self.position_reference = np.array(self.mat_file_h5py.get('position_reference')).transpose()
        self.link_position = np.array(self.mat_file_h5py.get('link_position')).transpose()
        self.motor_position = np.array(self.mat_file_h5py.get('motor_position')).transpose()

        self.velocity_reference = np.array(self.mat_file_h5py.get('velocity_reference')).transpose()
        self.link_velocity = np.array(self.mat_file_h5py.get('link_velocity')).transpose()
        self.motor_velocity = np.array(self.mat_file_h5py.get('motor_velocity')).transpose()

        self.effort = np.array(self.mat_file_h5py.get('effort')).transpose()
        self.effort_reference = np.array(self.mat_file_h5py.get('effort_reference')).transpose()

        self.temperature_motor = np.array(self.mat_file_h5py.get('temperature_motor')).transpose()
        self.temperature_driver = np.array(self.mat_file_h5py.get('temperature_driver')).transpose()

        self.stiffness = np.array(self.mat_file_h5py.get('stiffness')).transpose()
        self.damping = np.array(self.mat_file_h5py.get('damping')).transpose()

        self.fault = np.array(self.mat_file_h5py.get('fault')).transpose()
        
        # Relative time vectors (w.r.t. an absolute reference)
        self.abs_t0=self.abs_time[0] 
        self.aux_rel_time=self.aux_abs_time-self.abs_t0
        self.js_rel_time=self.abs_time-self.abs_t0


    ## Low-level methods for manipulating the .mat file ##

    def __read_str_from_ds(self, ds, index): 

        """

        Reads a string of a string cell array from a h5py database, given the index to be read. Only works with one-dimensional cell array of strings.
        
        Args:
            ds:
            index:

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
            name:

        """

        if 'aux_code' in name: # assigns the aux names and their associated codes to a dictionary

            self.aux_signals[name] = np.array(self.mat_file_h5py[name]).astype(int).flatten()[0]   # add the pair key-value to the dictionary
            
        if 'joint_names' in name: # assigns the joint names to a dictionary

            ds=self.mat_file_h5py[name] # extract a "dataset" from the loaded file

            for i in range(ds.size):
                self.joints[self.__read_str_from_ds(ds, i)] = i+1 # assign an incremental code to each joint, based on how they are already ordered

        if 'chain_names' in name: # assigns the chain names to a dictionary

            ds = self.mat_file_h5py[name] # extract a "dataset" from the loaded file

            for i in range(ds.size):
                self.chains[self.__read_str_from_ds(ds, i)] = np.array(self.mat_file_h5py[self.__read_str_from_ds(ds, i)]).astype(int).flatten()

        return None # any other value != to None would block the execution of visit() method

    ## Low-level methods for getting specific "raw" attributes of the class ##

    def get_joints(self):

        """

        Returns the joint dictionary (KEY: joint name -> VAL: joint code).

        """

        return self.joints

    def get_chains(self):

        """

        Returns the chain dictionary (KEY: chain name -> VAL: list of joint IDs belonging to the chain).

        """

        return self.chains

    def get_joint_names(self):
        
        """

        Returns ALL the joint names, as a list.

        """

        return list(self.joints.keys())

    def get_chain_names(self):

        """

        Returns the ALL the chain names, as a list.

        """

        return list(self.chains.keys())


    def get_raw_h5_file(self):

        """

        Returns the raw h5 object, created on the base of the loaded .mat file.

        """

        return self.mat_file_h5py


    def get_abs_t0(self):

        """

        Returns the absolute reference time (system time), used as a reference for computing the relative time vectors.

        """

        return self.abs_t0

    def get_js_seq(self):

        """

        Returns the sequence of packets of the joint state message.

        """

        return self.seq

    def get_js_abs_time(self):

        """

        Returns the absolute time vector (system time) associated with the joint state message.

        """

        return self.abs_time
    
    def get_js_rel_time(self):

        """

        Returns the relative time vector (w.r.t. abs_t0) associated with the joint state message.

        """

        return self.js_rel_time
    
    def get_aux_seq(self):

        """

        Returns the sequence of packets of the auxiliary state message.

        """

        return self.aux_seq

    def get_aux_abs_time(self):

        """

        Returns the absolute time vector (system time) associated with the auxiliary state message.

        """

        return self.aux_abs_time
    
    def get_aux_rel_time(self):

        """

        Returns the relative time vector (w.r.t. abs_t0) associated with the auxiliary state message.

        """

        return self.aux_rel_time


    def get_aux_signals(self):

        """

        Returns the auxiliary state dictionary (KEY: signal name -> VAL: signal code).

        """

        return self.aux_signals

    def get_aux_signal_names(self):

        """

        Returns ALL the signal names associated with the auxiliary state.

        """
        return list(self.aux_signals.keys())

    def get_aux_signal_codes(self, aux_signal_names):

        """
        Retrieve auxiliary state signals unique ID codes, given a list of valid signal names.

        Args:
            aux_signal_names: a list of valid signal names (which can be checked using the get_aux_signal_names() method).

        """

        aux_signal_codes = []

        try:

            for i in range(len(aux_signal_names)):
                aux_signal_codes.append(self.aux_signals[aux_signal_names[i]])

        except:

            print("ERROR: Could not get chains' auxiliary signal codes from the provided input. \n") 


        return aux_signal_codes
    
    def get_aux_types(self, jnt_id_list=None):

        """
        Retrieve one or more rows from the aux_types matrix of the auxiliary state message, given a valid list of joint IDs.
        If no ID list is provided, by default the whole matrix is returned.

        Args:
            jnt_id_list (optional): a list of valid signal IDs (which can be checked using the get_aux_signal_codes() method).

        """

        _, ncols = self.aux_type.shape

        if jnt_id_list is None:
            jnt_id_list = self.get_joints_id_from_names(self.get_joint_names())
        
        id_list_dim = len(jnt_id_list)

        jnt_indexes = np.array(jnt_id_list)-1 # MATLAB 1-BASED INDEXING

        extr_types = np.zeros((id_list_dim, ncols))

        for i in range(id_list_dim):
            extr_types[jnt_indexes[i]][:] = self.aux_type[i][:]
        
        return extr_types
    
    def get_aux_values(self, jnt_id_list=None):

        """
        Retrieve one or more rows from the aux_values matrix of the auxiliary state message, given a valid list of joint IDs.
        If no ID list is provided, by default the whole matrix is returned.

        Args:
            jnt_id_list (optional): a list of valid signal IDs (which can be checked using the get_aux_signal_codes() method).

        """

        _, ncols = self.aux_values.shape

        if jnt_id_list is None:
            jnt_id_list = self.get_joints_id_from_names(self.get_joint_names())
        
        id_list_dim =len(jnt_id_list)

        jnt_indexes = np.array(jnt_id_list)-1 # MATLAB 1-BASED INDEXING

        extr_values = np.zeros((id_list_dim, ncols))

        for i in range(id_list_dim):
            extr_values[jnt_indexes[i]][:] = self.aux_values[i][:]

        return extr_values
    
    def get_pos_references(self, jnt_id_list=None):
        
        """
        Retrieve one or more rows from the position_reference matrix of the joint state message, given a valid list of joint IDs.
        If no ID list is provided, by default the whole matrix is returned.

        Args:
            jnt_id_list (optional): a list of valid signal IDs (which can be checked using the get_aux_signal_codes() method).

        """

        _, aux_type_ncols = self.position_reference.shape

        if jnt_id_list is None:
            jnt_id_list = self.get_joints_id_from_names(self.get_joint_names())
        
        id_list_dim=  len(jnt_id_list)

        jnt_indexes = np.array(jnt_id_list)-1 # MATLAB 1-BASED INDEXING

        extr_values = np.zeros((id_list_dim, aux_type_ncols))

        for i in range(id_list_dim):
            extr_values[jnt_indexes[i]][:] = self.position_reference[i][:]

        return extr_values

    def get_links_position(self, jnt_id_list=None):
        
        """
        Retrieve one or more rows from the link_position matrix of the joint state message, given a valid list of joint IDs.
        If no ID list is provided, by default the whole matrix is returned.

        Args:
            jnt_id_list (optional): a list of valid signal IDs (which can be checked using the get_aux_signal_codes() method).

        """

        _, aux_type_ncols = self.link_position.shape

        if jnt_id_list is None:
            jnt_id_list = self.get_joints_id_from_names(self.get_joint_names())
        
        id_list_dim = len(jnt_id_list)

        jnt_indexes = np.array(jnt_id_list)-1 # MATLAB 1-BASED INDEXING

        extr_values = np.zeros((id_list_dim, aux_type_ncols))

        for i in range(id_list_dim):
            extr_values[jnt_indexes[i]][:] = self.link_position[i][:]

        return extr_values
    
    def get_motors_position(self, jnt_id_list=None):

        """
        Retrieve one or more rows from the motor_position matrix of the joint state message, given a valid list of joint IDs.
        If no ID list is provided, by default the whole matrix is returned.

        Args:
            jnt_id_list (optional): a list of valid signal IDs (which can be checked using the get_aux_signal_codes() method).

        """

        _, aux_type_ncols = self.motor_position.shape

        if jnt_id_list is None:
            jnt_id_list = self.get_joints_id_from_names(self.get_joint_names())
        
        id_list_dim = len(jnt_id_list)

        jnt_indexes = np.array(jnt_id_list)-1 # MATLAB 1-BASED INDEXING

        extr_values = np.zeros((id_list_dim, aux_type_ncols))

        for i in range(id_list_dim):
            extr_values[jnt_indexes[i]][:] = self.motor_position[i][:]

        return extr_values
    
    def get_velocity_references(self, jnt_id_list=None):

        """
        Retrieve one or more rows from the velocity_reference matrix of the joint state message, given a valid list of joint IDs.
        If no ID list is provided, by default the whole matrix is returned.

        Args:
            jnt_id_list (optional): a list of valid signal IDs (which can be checked using the get_aux_signal_codes() method).

        """

        _, aux_type_ncols = self.velocity_reference.shape

        if jnt_id_list is None:
            jnt_id_list = self.get_joints_id_from_names(self.get_joint_names())
        
        id_list_dim = len(jnt_id_list)

        jnt_indexes = np.array(jnt_id_list)-1 # MATLAB 1-BASED INDEXING

        extr_values = np.zeros((id_list_dim, aux_type_ncols))

        for i in range(id_list_dim):
            extr_values[jnt_indexes[i]][:] = self.velocity_reference[i][:]

        return extr_values

    def get_links_velocity(self, jnt_id_list=None):


        """
        Retrieve one or more rows from the link_velocity matrix of the joint state message, given a valid list of joint IDs.
        If no ID list is provided, by default the whole matrix is returned.

        Args:
            jnt_id_list (optional): a list of valid signal IDs (which can be checked using the get_aux_signal_codes() method).

        """

        _, aux_type_ncols = self.link_velocity.shape

        if jnt_id_list is None:
            jnt_id_list = self.get_joints_id_from_names(self.get_joint_names())
        
        id_list_dim = len(jnt_id_list)

        jnt_indexes = np.array(jnt_id_list)-1 # MATLAB 1-BASED INDEXING

        extr_values = np.zeros((id_list_dim, aux_type_ncols))

        for i in range(id_list_dim):
            extr_values[jnt_indexes[i]][:] = self.link_velocity[i][:]

        return extr_values
    
    def get_motors_velocity(self, jnt_id_list=None):

        """
        Retrieve one or more rows from the motor_velocity matrix of the joint state message, given a valid list of joint IDs.
        If no ID list is provided, by default the whole matrix is returned.

        Args:
            jnt_id_list (optional): a list of valid signal IDs (which can be checked using the get_aux_signal_codes() method).

        """

        _, aux_type_ncols = self.motor_velocity.shape

        if jnt_id_list is None:
            jnt_id_list = self.get_joints_id_from_names(self.get_joint_names())
        
        id_list_dim = len(jnt_id_list)

        jnt_indexes = np.array(jnt_id_list)-1 # MATLAB 1-BASED INDEXING

        extr_values = np.zeros((id_list_dim, aux_type_ncols))

        for i in range(id_list_dim):
            extr_values[jnt_indexes[i]][:] = self.motor_velocity[i][:]

        return extr_values
    
    def get_joints_efforts(self, jnt_id_list=None):

        """
        Retrieve one or more rows from the effort matrix of the joint state message, given a valid list of joint IDs.
        If no ID list is provided, by default the whole matrix is returned.

        Args:
            jnt_id_list (optional): a list of valid signal IDs (which can be checked using the get_aux_signal_codes() method).

        """

        _, aux_type_ncols = self.effort.shape

        if jnt_id_list is None:
            jnt_id_list = self.get_joints_id_from_names(self.get_joint_names())
        
        id_list_dim = len(jnt_id_list)

        jnt_indexes = np.array(jnt_id_list)-1 # MATLAB 1-BASED INDEXING

        extr_values = np.zeros((id_list_dim, aux_type_ncols))

        for i in range(id_list_dim):
            extr_values[jnt_indexes[i]][:] = self.effort[i][:]

        return extr_values
    
    def get_joints_efforts_ref(self, jnt_id_list=None):

        """
        Retrieve one or more rows from the effort_reference matrix of the joint state message, given a valid list of joint IDs.
        If no ID list is provided, by default the whole matrix is returned.

        Args:
            jnt_id_list (optional): a list of valid signal IDs (which can be checked using the get_aux_signal_codes() method).

        """

        _, aux_type_ncols = self.effort_reference.shape

        if jnt_id_list is None:
            jnt_id_list = self.get_joints_id_from_names(self.get_joint_names())
        
        id_list_dim = len(jnt_id_list)

        jnt_indexes = np.array(jnt_id_list)-1 # MATLAB 1-BASED INDEXING

        extr_values = np.zeros((id_list_dim, aux_type_ncols))

        for i in range(id_list_dim):
            extr_values[jnt_indexes[i]][:] = self.effort_reference[i][:]

        return extr_values
    
    def get_temp_motors(self, jnt_id_list=None):

        """
        Retrieve one or more rows from the temperature_motor matrix of the joint state message, given a valid list of joint IDs.
        If no ID list is provided, by default the whole matrix is returned.

        Args:
            jnt_id_list (optional): a list of valid signal IDs (which can be checked using the get_aux_signal_codes() method).

        """

        _, aux_type_ncols = self.temperature_motor.shape

        if jnt_id_list is None:
            jnt_id_list = self.get_joints_id_from_names(self.get_joint_names())
        
        id_list_dim = len(jnt_id_list)

        jnt_indexes = np.array(jnt_id_list)-1 # MATLAB 1-BASED INDEXING

        extr_values = np.zeros((id_list_dim, aux_type_ncols))

        for i in range(id_list_dim):
            extr_values[jnt_indexes[i]][:] = self.temperature_motor[i][:]

        return extr_values
    
    def get_temp_drivers(self, jnt_id_list=None):

        """
        Retrieve one or more rows from the temperature_driver matrix of the joint state message, given a valid list of joint IDs.
        If no ID list is provided, by default the whole matrix is returned.

        Args:
            jnt_id_list (optional): a list of valid signal IDs (which can be checked using the get_aux_signal_codes() method).

        """

        _, aux_type_ncols = self.temperature_driver.shape

        if jnt_id_list is None:
            jnt_id_list = self.get_joints_id_from_names(self.get_joint_names())
        
        id_list_dim = len(jnt_id_list)

        jnt_indexes = np.array(jnt_id_list)-1 # MATLAB 1-BASED INDEXING

        extr_values = np.zeros((id_list_dim, aux_type_ncols))

        for i in range(id_list_dim):
            extr_values[jnt_indexes[i]][:] = self.temperature_driver[i][:]

        return extr_values
    
    def get_joints_stiffness(self, jnt_id_list=None):

        """
        Retrieve one or more rows from the stiffness matrix of the joint state message, given a valid list of joint IDs.
        If no ID list is provided, by default the whole matrix is returned.

        Args:
            jnt_id_list (optional): a list of valid signal IDs (which can be checked using the get_aux_signal_codes() method).

        """

        _, aux_type_ncols = self.stiffness.shape

        if jnt_id_list is None:
            jnt_id_list = self.get_joints_id_from_names(self.get_joint_names())
        
        id_list_dim = len(jnt_id_list)

        jnt_indexes = np.array(jnt_id_list)-1 # MATLAB 1-BASED INDEXING

        extr_values = np.zeros((id_list_dim, aux_type_ncols))

        for i in range(id_list_dim):
            extr_values[jnt_indexes[i]][:] = self.stiffness[i][:]

        return extr_values
    
    def get_joints_damping(self, jnt_id_list=None):

        """
        Retrieve one or more rows from the damping matrix of the joint state message, given a valid list of joint IDs.
        If no ID list is provided, by default the whole matrix is returned.

        Args:
            jnt_id_list (optional): a list of valid signal IDs (which can be checked using the get_aux_signal_codes() method).

        """

        _, aux_type_ncols = self.damping.shape

        if jnt_id_list is None:
            jnt_id_list = self.get_joints_id_from_names(self.get_joint_names())
        
        id_list_dim = len(jnt_id_list)

        jnt_indexes = np.array(jnt_id_list)-1 # MATLAB 1-BASED INDEXING

        extr_values = np.zeros((id_list_dim, aux_type_ncols))

        for i in range(id_list_dim):
            extr_values[jnt_indexes[i]][:] = self.damping[i][:]

        return extr_values
    
    def get_faults(self, jnt_id_list=None):

        """
        Retrieve one or more rows from the fault matrix of the joint state message, given a valid list of joint IDs.
        If no ID list is provided, by default the whole matrix is returned.

        Args:
            jnt_id_list (optional): a list of valid signal IDs (which can be checked using the get_aux_signal_codes() method).

        """

        _, aux_type_ncols = self.fault.shape

        if jnt_id_list is None:
            jnt_id_list = self.get_joints_id_from_names(self.get_joint_names())
        
        id_list_dim = len(jnt_id_list)

        jnt_indexes = np.array(jnt_id_list)-1 # MATLAB 1-BASED INDEXING

        extr_values = np.zeros((id_list_dim, aux_type_ncols))

        for i in range(id_list_dim):
            extr_values[jnt_indexes[i]][:] = self.fault[i][:]

        return extr_values
 
    ## High-level methods ##

    def get_joint_names_from_id(self, id_list):
        
        """

        Retrieve joints' unique names, given a joint ID.
        If the input is provided as a list of joint IDs, the method will return a list of joint names.
        
        Args:
            id_list: a list of valid joint IDs (which can be checked using the get_joints_id_from_names() method)

        """

        jnt_names_list = []

        # id_list_flattened=list(chain.from_iterable(id_list)) # flatten input list in case it is 2D

        name_keys = self.joints.keys()
        id_values = self.joints.values()

        try:
            for i in range(len(id_list)):
                id_index = list(id_values).index(id_list[i])
                jnt_names_list.append(list(name_keys)[id_index])

        except:

            print("ERROR: Could not get joint names from the provided input. \n") 

        return jnt_names_list
    
    def get_joints_id_from_names(self, jnt_name_list):

        """

        Retrieve joints' unique IDs, given a joint name (or list).
        If the input is provided as a list of joint names, the method will return a list of joint IDs.
        
        Args:
            jnt_name_list: list of valid joint names (which can be checked using the get_joint_names() method)

        """

        jnt_ids_list = []
        
        try:
            for i in range(len(jnt_name_list)):
                jnt_ids_list.append(self.joints[jnt_name_list[i]])

        except:
            print("ERROR: Could not get joint IDs from the provided input. \n") 

        return jnt_ids_list

    def get_chains_jnt_id(self, chain_name_list):

        """

        Retrieve joints' unique IDs belonging to a particular chain (or list of chains).
        In case the input is a list of chain names, the output will be a 2D list of joint IDs.
        
        Args:
            chain_name_list: a list of valid chain names (which can be checked using the get_chain_names() method)
        
        """

        chains_jnt_id = []

        try:
            for i in range(len(chain_name_list)):
                chains_jnt_id.append(list(self.chains[chain_name_list[i]]))

        except:
            print("ERROR: Could not get chains' joint IDs from the provided input. \n") 


        return chains_jnt_id
    
    def extr_from_aux(self, jnt_index, sgnl_name):

        """

        Extracts the required information from the aux_values matrix and aux_time vector, based on the provided joint index and signal name.
        
        Args:
            jnt_index: a valid joint ID (int)
            sgnl_name: a valid auxiliary signal name
        
        """

        sgnl_code = self.get_aux_signal_codes([sgnl_name])
        sgn_code_selector = self.get_aux_types()[jnt_index][:] == sgnl_code
        aux_time_extr = self.get_aux_rel_time()[sgn_code_selector]
        aux_vals_extr = self.get_aux_values()[jnt_index][sgn_code_selector]

        return aux_time_extr, aux_vals_extr

################################### LOG PLOTTER class ###################################

class LogPlotter(): 

    def __init__(self, log_loader, clr_set_name= "Set1"):
        
        """
        Contructor
        Args:
            log_loader --> LogLoader object
            clr_set_name --> colors set name used for plotting lines on the same axis.
                             For other set types see https://matplotlib.org/stable/gallery/color/colormap_reference.html
        
        """

        self.fig={} # dictionary containing all the opened figures
        self.axes={} # fig. name -> fig. axes

        self.lines=[] # auxiliary object to allow the use of legend picking on subsequent calls to js_plot() and aux_plot()

        # self.matfile_path=matfile_path
        self.log_loader=log_loader
       
        self.clr_set_name = clr_set_name # 
        
        self.lined={}
    

    def init_fig(self, fig_name):

        """

        Initialize a figure.

        Args:
            fig_name: figure name (used to reference the correct figure objects).

        """

        fig= plt.figure()
        self.fig[fig_name]=fig
        
        return fig

    def add_subplot(self, fig_name, row, column, index):

        """

        Add a subplot to the selected figure.

        Args:
            fig_name: figure name.
            row:
            column:
            index:

        """

        subplt_axes=self.fig[fig_name].add_subplot(row, column, index)

        cmap = get_cmap(self.clr_set_name) 
        colors = cmap.colors 
        subplt_axes.set_prop_cycle(color=colors)

        self.axes[fig_name]=subplt_axes # updating the axes

    def aux_plot(self, fig_name, jnt_id, leg_draggable=True, title=None, set_grid=True,
                 draw_style='default', enable_lgnd_pick=True, add_plot=False):
        
        """

        Plot every auxiliary signal associated with a given joint ID.

        Args:
            fig_name: figure name.
            jnt_id:
            leg_draggable:
            title:
            set_grid:
            draw_style:
            enable_lgnd_pick:
            add_plot:

        """

        jnt_index=jnt_id-1 # MATLAB INDEXING
        if (not add_plot):
            self.lines=[] # clear aux auxiliary attribute (only necessary to make legend picking work properly)

        for sgnl_name in self.log_loader.get_aux_signal_names():

            aux_time_extr, aux_vals_extr= self.log_loader.extr_from_aux(jnt_index, sgnl_name)

            self.lines.append(self.axes[fig_name].plot(aux_time_extr, aux_vals_extr, label=sgnl_name.replace('_aux_code', ''), drawstyle=draw_style)) # plotting on the last active subplot

        if (set_grid): self.axes[fig_name].grid()
        if (title is not None): self.axes[fig_name].set_title(title)

        legend =self.axes[fig_name].legend(fancybox=True, shadow=False)
        legend.set_draggable(leg_draggable)
        
        if (enable_lgnd_pick):

                for legline, origline in zip(legend.get_lines(), self.lines):
                    legline.set_picker(5)  # 5 pts tolerance
                    self.lined[legline] = origline

                self.fig[fig_name].canvas.mpl_connect('pick_event', self.onpick)

        self.fig[fig_name].show()

    def js_plot(self, fig_name, jnt_id, input_matrix, x_data=None, leg_draggable=True, title=None, line_label=None, set_grid=True,
                draw_style='default', enable_lgnd_pick=True, add_plot=False):

        """

        Given an input matrix (n_joints x n_samples), plot the data of the row associated with the selected joint ID.

        Args:
            fig_name: figure name.
            jnt_id:
            input_matrix:
            leg_draggable:
            title:
            line_label:
            set_grid:
            draw_style:
            enable_lgnd_pick:
            add_plot:

        """

        jnt_index=jnt_id-1 # MATLAB INDEXING
        jnt_name=self.log_loader.get_joint_names_from_id([jnt_id])[0]

        if (not add_plot):
            self.lines=[] # clear js auxiliary attribute (only necessary to make legend picking work properly)

        label_plt=jnt_name + " joint"
        if (line_label is not None): label_plt=line_label

        time=self.log_loader.get_js_rel_time()
        if (x_data is not None): time= x_data
        
        vals=input_matrix[jnt_index][:]
        self.lines.append(self.axes[fig_name].plot(time, vals, label=label_plt, drawstyle=draw_style)) # plotting on the last active subplot

        if (set_grid): self.axes[fig_name].grid()
        if (title is not None): self.axes[fig_name].set_title(title)
        
        legend =self.axes[fig_name].legend(fancybox=True, shadow=False)
        legend.set_draggable(leg_draggable)

        if (enable_lgnd_pick):

                for legline, origline in zip(legend.get_lines(), self.lines):
                    legline.set_picker(5)  # 5 pts tolerance
                    self.lined[legline] = origline

                self.fig[fig_name].canvas.mpl_connect('pick_event', self.onpick)

        self.fig[fig_name].show()
        

    def onpick(self, event):

        """

        On the pick event, find the orig line corresponding to the legend proxy line, and toggle its visibility.

        """
        
        if isinstance(event.artist, Line2D):
            legendline = event.artist
            originline = self.lined[legendline]
            visible = not originline[0].get_visible()
            originline[0].set_visible(visible)
            legendline.set_alpha(1.0 if visible else 0.2)
            event.canvas.draw()
        
def main():

    # EXAMPLE USAGE (Centauro log file)

    matfile_path = rospkg.RosPack().get_path("xbot_ros")+'/load_folder/'+'centauro_robot_state_log.mat' # path to the .mat file

    log_loader = LogLoader(matfile_path) # initializing the LogLoader for reading and using the data inside the .mat file
    
    canvas = LogPlotter(log_loader, clr_set_name="tab10") # plotter instance

    n_rows1 = 2 # subplot rows
    n_cols1 = 1 # subplot columns
    
    canvas.init_fig(fig_name="prova1") # initialize the figure
    
    canvas.add_subplot(fig_name = "prova1", row=n_rows1, column=n_cols1, index=1) # adding a subplot in the specified position
    canvas.aux_plot(fig_name = "prova1", jnt_id=1, title="Auxiliary state signals on joint \""+ canvas.log_loader.get_joint_names_from_id([1])[0]+"\"") # plotting the aux signals of a particular joint
    
    canvas.add_subplot(fig_name =  "prova1", row = n_rows1, column = n_cols1, index = 2) # adding another subplot
    canvas.js_plot(fig_name = "prova1", input_matrix = log_loader.get_joints_efforts(), jnt_id = 12, line_label = log_loader.get_joint_names_from_id([12])[0]+" torque", title = "Some joint torques") # plotting the torque on a specified joint
    canvas.js_plot(fig_name = "prova1", input_matrix = log_loader.get_joints_efforts(), jnt_id = 15, set_grid = False, add_plot = True, line_label = log_loader.get_joint_names_from_id([15])[0]+" torque") # another torque plot

    input() # necessary to keep all figures opened

if __name__=="__main__":

    main()