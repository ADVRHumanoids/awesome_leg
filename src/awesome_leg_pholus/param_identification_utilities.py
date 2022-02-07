#!/usr/bin/env python3

from re import I
from xbot_interface import config_options as opt

import numpy as np

from scipy import interpolate

import casadi as cas

import pinocchio as pin

######################### MISCELLANEOUS UTILITIES DEFINITIONS #########################

# FRICTION MODEL FOR I_Q:

def interp(time_vector, vect_vals, axis = None):

    """
        Interpolate vect_vals vector over the provided time_vector.
        By default interpolate along axis 0 

        Args:
            time_vector: time vector over which to interpolate the input
            vect_vals: input time vector to be interpolated
            axis: axis along which the input will be interpolated 

    """

    if axis == None:
        axis = 0

    f = interpolate.interp1d(time_vector, vect_vals, axis)

    return f

# def compute_P_qT(i_q_time, js_time, i_q, q_p_dot, q_p_ddot, tau_link_meas, red_ratios, N_samples, vel_sign_threshold):
     
#     n_opt_var = 4 # number of optimization var per joint

#     t0_max = np.amax(np.array([ i_q_time[0, 1], i_q_time[1, 1], js_time[1] ])) # maximum value between the seconds elements of the input time arrays
#     tf_min = np.amin(np.array([ i_q_time[0, -2], i_q_time[1, -2], js_time[-2] ])) # minimum value between the penultimate elements of the input time arrays

#     ref_time_margin= 1e-6 # used to prevent numerical errors from making the interpolation explode
#     t0_max = t0_max + ref_time_margin # shifting up the lower value of the ref time by ref_time_margin
#     tf_min = tf_min - ref_time_margin # shifting down the upper value of the ref time by ref_time_margin

#     dt_samples= (tf_min - t0_max)/(N_samples - 1)
#     ref_time = np.zeros((N_samples))
#     ref_time[0] = t0_max
#     for i in range(N_samples-1):
#         ref_time[i+1] = ref_time[i] + dt_samples


#     f_q_p_dot = interp(js_time, q_p_dot)
#     q_p_dot_interp = f_q_p_dot(ref_time)
  
#     f_q_p_ddot = interp(js_time[1: len(js_time)], q_p_ddot)
#     q_p_ddot_interp = f_q_p_ddot(ref_time)

#     f_tau = interp(js_time, tau_link_meas)
#     tau_interp = f_tau(ref_time)

#     f_i_q_hip = interp(i_q_time[0, :], i_q[0, :], axis = 0)
#     f_i_q_knee = interp(i_q_time[1, :], i_q[1, :], axis = 0)
#     i_q_hip_interp = f_i_q_hip(ref_time)
#     i_q_knee_interp = f_i_q_knee(ref_time)
#     i_q_interp=np.array([i_q_hip_interp, i_q_knee_interp])
    

#     vel_signs= compute_sign_mat(q_p_dot_interp, threshold = vel_sign_threshold) # vel values below

#     rows = N_samples 
#     cols = n_opt_var
#     A_hip = np.zeros((rows, cols))
#     A_knee = np.zeros((rows, cols))

#     B_hip=np.zeros((rows, 1))
#     B_knee=np.zeros((rows, 1))
#     zero_matrix=np.zeros((rows, cols)) # used to stack the two independent hip and knee problems

#     for i in range(0, N_samples):

#         B_hip[i] = - red_ratios[0] * tau_interp[0, i]
#         B_knee[i] = - red_ratios[1] * tau_interp[1, i]

#         A_hip[i , 0] = q_p_ddot_interp[0, i]/ red_ratios[0] 
#         A_hip[i , 1] = - i_q_interp[0, i] 
#         A_hip[i , 2] = red_ratios[0] * vel_signs[0, i]
#         A_hip[i , 3] = red_ratios[0] * q_p_dot_interp[0, i]

#         A_knee[i , 0] = q_p_ddot_interp[1, i]/ red_ratios[1] 
#         A_knee[i , 1] = - i_q_interp[1, i] 
#         A_knee[i , 2] = red_ratios[1] * vel_signs[1, i]
#         A_knee[i , 3] = red_ratios[1] * q_p_dot_interp[1, i]

#     first_stack = np.concatenate((A_hip, zero_matrix), axis=1)
#     second_stack = np.concatenate((zero_matrix, A_knee), axis=1)
#     A_tot = np.concatenate((first_stack, second_stack), axis=0)
#     B_tot= np.concatenate((B_hip, B_knee), axis = 0)
    
#     P = np.dot(A_tot.T, A_tot)
#     q_T= - np.dot(B_tot.T, A_tot)
    
#     return P, q_T.flatten(), A_tot, B_tot

def compute_P_qT_i_q(sigma, X_guess, jnt_index, i_q_time, js_time, i_q, q_p_dot, q_p_ddot, tau_link_meas, red_ratios, N_samples, vel_sign_threshold, smooth_coeff):
    
    """
        Compute the P and q_T needed by the QP of the i_q model identification, for a single joint.
        

        Args:
            sigma: regularization matrix
            X_guess: guess solution vector to be used by the regularization term
            jnt_index: joint index
            i_q_time: time list with the following structure [jnt_index][n_samples]; not a numpy array because the n. samples can be, in general, different between joints
            js_time: joint state time vector
            i_q: i_q list with the following structure [jnt_index][n_samples]; not a numpy array because the n. samples can be, in general, different between joints
            q_p_dot: joint velocity
            q_p_ddot: joint acceleration
            tau_link_meas: measured link torques
            red_ratios: reduction ratio vector (n_jnts x 1)
            N_samples: number of samples over which the interpolation will happen (same as the QP number of samples)
            vel_sign_threshold: parameter used if employing an "hard" sign function. Threshold under which the return value is 0
            smooth_coeff: parameter used by the "smooth" sign function. The higher the parameter, the steeper the transition between 0 and 1 will be.

    """

    t0_max = np.amax(np.array([ i_q_time[jnt_index][1], js_time[1] ])) # maximum value between the seconds elements of the input time arrays 
    # (1 index necessary to avoid problems with the interpolation of the differentiated joint acceleration)
    tf_min = np.amin(np.array([ i_q_time[jnt_index][-2], js_time[-2] ])) # minimum value between the penultimate elements of the input time arrays
    # (1 index necessary to avoid problems with the interpolation of the differentiated joint acceleration)
    ref_time_margin= 1e-6 # used to prevent numerical errors from making the interpolation explode
    t0_max = t0_max + ref_time_margin # shifting up the lower value of the ref time by ref_time_margin
    tf_min = tf_min - ref_time_margin # shifting down the upper value of the ref time by ref_time_margin

    dt_samples= (tf_min - t0_max) / (N_samples - 1)
    ref_time = np.zeros((N_samples))
    ref_time[0] = t0_max
    for i in range(N_samples-1):
        ref_time[i+1] = ref_time[i] + dt_samples

    f_q_p_dot = interp(js_time, q_p_dot[jnt_index, :])
    q_p_dot_interp = f_q_p_dot(ref_time)
  
    f_q_p_ddot = interp(js_time[1: len(js_time)], q_p_ddot[jnt_index, :])
    q_p_ddot_interp = f_q_p_ddot(ref_time)

    f_tau = interp(js_time, tau_link_meas[jnt_index, :])
    tau_interp = f_tau(ref_time)

    f_i_q = interp(i_q_time[jnt_index][:], i_q[jnt_index][:])
    i_q_interp = f_i_q(ref_time)

    # vel_signs = compute_sign_vect(q_p_dot_interp, threshold = vel_sign_threshold) # vel values below
    vel_signs= compute_smooth_sign(q_p_dot_interp, coeff = smooth_coeff)

    n_opt_var = 4 # number of optimization vars per joint (axial_MoI, Kt, K_d0, K_d1)
    rows = N_samples 
    cols = n_opt_var

    A = np.zeros((rows, cols))
    B = np.zeros((rows, 1)).flatten()

    for i in range(0, N_samples):

        # if vel_signs[i] == 0: # within the prescribed velocity threshold, nullify the contribution of the sample to the cost function

        #     B[i] = 0

        #     A[i , 0] = 0
        #     A[i , 1] = 0 
        #     A[i , 2] = 0
        #     A[i , 3] = 0

        # else:

        B[i] = - red_ratios[jnt_index] * tau_interp[i]

        A[i , 0] = q_p_ddot_interp[i]/ red_ratios[jnt_index] 
        A[i , 1] = - i_q_interp[i] 
        A[i , 2] = red_ratios[jnt_index] * vel_signs[i]
        A[i , 3] = red_ratios[jnt_index] * q_p_dot_interp[i]
    
    P = np.dot(A.T, A)
    q_T = - np.dot(B.T, A) - np.dot(np.transpose(X_guess),np.dot(np.transpose(sigma), sigma))

    return P, q_T.flatten(), A, B

def compute_cost(x, P, q_T, B):

    """
        Compute the cost, given a x, P, q_T, B

        Args:
            x: solution vector 
            P: quadratic term matrix (1/2 x^T P x)
            q_T: linear term vector q^T x
            B: "measurement vector" --> so that A X = B, where A is the regressor used to build P and q_T
           
    """
    cost = 1/2 * np.dot(x.T, np.dot(P, x)) + np.dot (q_T, x) + np.dot(B.T, B)/2
    return cost.flatten()[0]

def get_xbot_cfg(urdf, srdf, is_fb = False):

    """
    A function to construct a XBot config object, given the URDF and the SRDF of the robot.

    Args:
            urdf: urdf description of the robot
            srdf: srdf description of the robot
            is_fb: whether the robot is floating base or not

    """
    
    cfg = opt.ConfigOptions()
    cfg.set_urdf(urdf)
    cfg.set_srdf(srdf)
    cfg.generate_jidmap()
    cfg.set_bool_parameter('is_model_floating_base', is_fb)
    cfg.set_string_parameter('model_type', 'RBDL')
    cfg.set_string_parameter('framework', 'ROS')
    
    return cfg

def compute_tau_over_jnt_traj_xbot(model, q_p, q_p_dot, q_p_ddot):

    """
    A function to compute the efforts necessary to execute a given joint trajectory, using the xbot model interface.
    The input vectors must be formatted in the form n_joints x number_of_samples

    Returns a n_joint x n_samples numpy array.

    Args:
            model: XBot model
            q_p: joint positions (n_jnts x n_samples)
            q_p_dot: joint velocities (n_jnts x n_samples)
            q_p_ddot: joint accelerations (n_jnts x n_samples)
    
    """

    n_joints = len(q_p)
    n_samples = len(q_p[0])
    tau = np.zeros((n_joints, n_samples))

    for i in range(0, n_samples):

        model.setJointPosition(q_p[:,i])
        model.setJointVelocity(q_p_dot[:,i])
        model.setJointAcceleration(q_p_ddot[:,i])
        model.update()

        tau_aux = model.computeInverseDynamics()
        for j in range(0, n_joints):
            tau[j,i] = tau_aux[j]

    return tau

def compute_tau_over_traj_pin(model, data, q_p, q_p_dot, q_p_ddot):

    """
    A function to compute the efforts necessary to execute a given joint trajectory using Pinocchio.
    The input vectors must be formatted in the form n_joints x number_of_samples

    Returns a n_joint x n_samples numpy array.

    Args:
            model: Pinocchio model
            data: data object associated with the Pinocchio model
            q_p: joint positions (n_jnts x n_samples)
            q_p_dot: joint velocities (n_jnts x n_samples)
            q_p_ddot: joint accelerations (n_jnts x n_samples)
    
    """
    n_joints = len(q_p)
    n_samples = len(q_p[0])
    tau = np.zeros((n_joints, n_samples))

    for i in range(0, n_samples):

        tau_aux = pin.rnea(model, data, q_p[:,i], q_p_dot[:,i], q_p_ddot[:,i])

        for j in range(0, n_joints):
            tau[j,i] = tau_aux[j]

    return tau

def diff_mat(time, vect):

    """

    A function to compute the derivative of an input matrix over a given time vector.

    Returns a numpy array of dimension n_joints x (length(time)-1).

    Args:
            time: time vector over which the input vector will be differentiated
            vect: input vector
            
    """

    n_joints = len(vect)
    n_cols = len(vect[0])-1
    vect_dot = np.zeros((n_joints, n_cols))

    for i in range(0, n_cols-1):

        for j in range(0, n_joints):
            vect_dot[j, i] = (vect[j, i+1] - vect[j,i])/ (time[i+1] - time[i])
    
    return vect_dot

def compute_sign_mat(mat, threshold = None):

    """
    Function  to compute an "hard" sign function over a matrix.
    
    Args:
        mat: input matrix 
        threshold: sign threshold (below this the sign function returns 0)
            
    """

    if threshold is None:
        threshold = 1e-4

    n_samples = len(mat[0,:])
    n_jnts=  len(mat[:,0])

    sign_vect=np.zeros((n_jnts, n_samples))

    for i in range(0, n_samples):

        for j in range(0, n_jnts):

            if mat[j, i] > threshold:

                sign_vect[j, i] = 1

            elif mat[j, i] < - threshold:

                sign_vect[j, i] = - 1
            else:

                sign_vect[j, i] = 0

    return sign_vect

def compute_sign_vect(vect, threshold = None):

    """
    Function to compute an "hard" sign function over a vector.
    
    Args:
        vect: input vector 
        threshold: sign threshold (below this the sign function returns 0)
            
    """


    if threshold is None:
        threshold = 1e-4

    sign_vect=np.zeros(len(vect))

    for i in range(0, len(vect)):

        if vect[i] > threshold:
            sign_vect[i] = 1

        elif vect[i] < - threshold:
            sign_vect[i] = - 1

        else:
            sign_vect[i] = 0

    return sign_vect

def compute_smooth_sign(input, coeff):

    """

    Computes a smooth approximation of the sign function.

    Args:
        input: input over which to compute the sign
        coeff: coefficient of the tanh function used to approximate the sign

    """

    return np.tanh( coeff * input)

def compute_cost_raw(A, B, x):

    """

    Computes the regression cost of a problem of the type A *  x = B.

    Returns the scalar cost.

    Args:
        A: regressor
        B: measurement vector
        x: solution

    """

    error = np.dot(A, x) - np.transpose(B)

    cost = 1/2 * np.dot(np.transpose(error), error)

    return cost

def qpsolve_cas(H, g, lbx, ubx):

    """

    Solve basic QP with Casadi.

    Returns the solution X.

    Args:
        H: quadratic term matrix (1/2 x^T H x)
        g: linear term vector g^T x
        lbx: lower bounds
        ubx: upper bounds

    """

    # Convert to CasADi types
    H = cas.DM(H)
    g = cas.DM(g)
    lbx = cas.DM(lbx)
    ubx = cas.DM(ubx)

    # QP structure
    qp = {}
    qp['h'] = H.sparsity()

    # Create CasADi solver instance
    S = cas.conic('S','qpoases',qp)    

    r = S(h = H, g = g, lbx = lbx, ubx = ubx)
    
    X = np.array(r['x']).flatten()
    # Return the solution
    return X

def rearrange_test_mat(mat):

    """
    Simple function to rearrange an input matrix. Used when parsing toghether average data from different static tests.

    Args:
        mat: input matrix to be rearranged
    
    """

    n_jnts = len(mat[0])
    n_traj = len(mat)
    taus_new = np.zeros((n_jnts, n_traj))

    for i in range(n_jnts):
        for j in range(n_traj):
            taus_new[i, j] = mat[j][i]

    return taus_new

# def compute_l_CoM(g, m1, m2, l_hip, q_p, taus):
    
#     n_jnts = len(taus[:, 0])
#     n_traj = len(taus[0, :])
#     l_CoM = np.zeros((n_jnts, n_traj))    

#     for j in range(n_traj):

#         l_CoM[0, j] =  taus[0, j] / (m1 * g * (np.sin(q_p[1, j]-q_p[0, j])))
#         l_CoM[1, j] = ((taus[0,j]+taus[1,j]) / (g * np.sin(q_p[0, j]))- m2 * l_hip) / m1

#     return l_CoM

# INERTIAL PARAMS ESTIMATION:

def assemblePinocchioRegressor(model, data, q_p, q_p_dot, q_p_ddot):

    """
    Assemble the total inverse dynamics regressor for a robot.

    Args:
        model: Pinocchio model 
        data: Pinocchio data object, associated with the model
        q_p: joint positions
        q_p_dot: joint velocities
        q_p_ddot: joint accelerations
    
    """

    n_jnts = len(q_p[:, 0])
    n_samples = len(q_p[0, :])
    n_inertia_params = len(model.inertias[0].toDynamicParameters())
    regressors = np.zeros((n_jnts, n_samples, n_jnts * n_inertia_params + n_jnts))
    bias_matrix = np.eye(n_jnts)

    for i in range(n_samples):
        regressor_aux = pin.computeJointTorqueRegressor(model, data, q_p[:, i], q_p_dot[:, i], q_p_ddot[:, i])
        for j in range(n_jnts):
            regressors[j, i, :] = np.concatenate((regressor_aux[j, :], bias_matrix[j, :]), axis = 0)

    return regressors

def compute_P_qT_inertial(regressor, tau_meas, sigma, X_guess):

    """
    Compute the P and qT for the inertial identification problem.

    Args:
        regressor: regressor matrix (n_jnts x n_samples x X_dim)
        tau_meas: measured tau
        sigma: regularization matrix
        X_guess: guess solution used by the regularization matrix
    
    """

    n_jnts = len(tau_meas[:,0])
    regularization_quad = np.dot(np.transpose(sigma), sigma) # regularization quadratic term

    P = regularization_quad # initializing P with the regularization term
    q_T = - np.dot(np.transpose(X_guess),np.dot(np.transpose(sigma), sigma)) # initializing q_T with the regularization term

    for i in range(n_jnts): # adding contributions of each joint to the optimization matrix P
        P = P + np.dot(np.transpose(regressor[i,:,:]), regressor[i, :, :])
        q_T = q_T - np.dot(np.transpose(tau_meas[i, :]), regressor[i])

    return P, q_T

def compute_skew(vect):

    """
    Compute the skew representation of a vecor.

    Args:
        vect: input vector
    
    """

    skew = np.zeros((3, 3))

    skew[2, 1] = vect[0]
    skew[2, 0] = - vect[1]
    skew[1, 0] = vect[2]

    skew = skew - np.transpose(skew)

    return skew

def interpret_inertial_sol(n_active_jnts, X):

    """
    Given a solution of the inertial params regression problem, interpret Pinocchio data and return a "decoded" vector.

    Args:
        n_active_jnts: input vector
        X: solution vector of the  regression problem (n_jnts x (10 + 1))
    
    """

    # Please note: Pinocchio returns the MoI computed in the joint frame.
    # Also, the inertial vector from Pinocchio is structured in the following way:
    # [m, m * lg_x, m * lg_y, m * lg_z, Ixx, Ixy, Iyy, Ixz, Iyz, Izz]

    sol_length = (len(X))

    PI = X[0: (sol_length - n_active_jnts)] # inertial params as outputted by Pinocchio
    
    tau_tilde = X[(sol_length - n_active_jnts): sol_length] # torque bias estimate

    inertial_params = np.zeros(sol_length).flatten()

    inertial_params_offset = 10 # 1 + 3 + 6

    for i in range(n_active_jnts):

        mass = PI[i * inertial_params_offset]

        l_CoM_x = PI[i * inertial_params_offset + 1] / mass
        l_CoM_y = PI[i * inertial_params_offset + 2] / mass
        l_CoM_z = PI[i * inertial_params_offset + 3] / mass
        levers = np.array([l_CoM_x, l_CoM_y, l_CoM_z])
        S = compute_skew(levers) # skew matrix computation

        I = np.zeros((3, 3))
        I[0, 0] = PI[i * inertial_params_offset + 4]
        I[0, 1] = PI[i * inertial_params_offset + 5]
        I[1, 1] = PI[i * inertial_params_offset + 6]
        I[0, 2] = PI[i * inertial_params_offset + 7]
        I[1, 2] = PI[i * inertial_params_offset + 8]
        I[2, 2] = PI[i * inertial_params_offset + 9]
        I = I + np.transpose(I)

        I_c = I - mass * np.transpose(S) * S

        inertial_params[i * inertial_params_offset] = mass
        inertial_params[i * inertial_params_offset + 1] = l_CoM_x
        inertial_params[i * inertial_params_offset + 2] = l_CoM_y
        inertial_params[i * inertial_params_offset + 3] = l_CoM_z
        inertial_params[i * inertial_params_offset + 4] = I_c[0, 0] # ixx
        inertial_params[i * inertial_params_offset + 5] = I_c[0, 1] # ixy
        inertial_params[i * inertial_params_offset + 6] = I_c[0, 2] # ixz
        inertial_params[i * inertial_params_offset + 7] = I_c[1, 1] # iyy
        inertial_params[i * inertial_params_offset + 8] = I_c[1, 2] # iyz
        inertial_params[i * inertial_params_offset + 9] = I_c[2, 2] # izz

    return inertial_params, tau_tilde

def interpret_sol2(n_active_jnts, X):

    """
    Given a solution of the inertial params regression problem, interpret Pinocchio data and return a "decoded" vector.
    Second version (works element-wise)

    Args:
        n_active_jnts: input vector
        X: solution vector of the  regression problem (n_jnts x (10 + 1))
    
    """

    # Please note: Pinocchio returns the MoI computed in the joint frame.

    sol_length = (len(X))

    PI = X[0: (sol_length - n_active_jnts)] # inertial params as outputted by Pinocchio
    
    tau_tilde = X[(sol_length - n_active_jnts): sol_length] # torque bias estimate

    inertial_params = np.zeros(sol_length).flatten()

    inertial_params_offset = 10 # 1 + 3 + 6

    for i in range(n_active_jnts):

        mass = PI[i * inertial_params_offset]
        l_CoM_x = PI[i * inertial_params_offset + 1] / PI[i * inertial_params_offset] 
        l_CoM_y = PI[i * inertial_params_offset + 2] / PI[i * inertial_params_offset]
        l_CoM_z = PI[i * inertial_params_offset + 3] / PI[i * inertial_params_offset]
        I_xx = PI[i * inertial_params_offset + 4]
        I_xy = PI[i * inertial_params_offset + 5]
        I_yy = PI[i * inertial_params_offset + 6]
        I_xz = PI[i * inertial_params_offset + 7]
        I_yz = PI[i * inertial_params_offset + 8]
        I_zz = PI[i * inertial_params_offset + 9]

        inertial_params[i * inertial_params_offset] = mass
        inertial_params[i * inertial_params_offset + 1] = l_CoM_x
        inertial_params[i * inertial_params_offset + 2] = l_CoM_y
        inertial_params[i * inertial_params_offset + 3] = l_CoM_z
        inertial_params[i * inertial_params_offset + 4] = I_xx - mass * (np.power(l_CoM_y, 2) + np.power(l_CoM_z, 2)) # ixx
        inertial_params[i * inertial_params_offset + 5] = I_xy + mass * l_CoM_x * l_CoM_y # ixy
        inertial_params[i * inertial_params_offset + 6] = I_xz + mass * l_CoM_x * l_CoM_z # ixz
        inertial_params[i * inertial_params_offset + 7] = I_yy - mass * (np.power(l_CoM_x, 2) + np.power(l_CoM_z, 2)) # iyy
        inertial_params[i * inertial_params_offset + 8] = I_yz + mass * l_CoM_y * l_CoM_z # iyz
        inertial_params[i * inertial_params_offset + 9] = I_zz - mass * (np.power(l_CoM_x, 2) + np.power(l_CoM_y, 2)) # izz

    return inertial_params, tau_tilde

def compute_fitting_error(regressors, tau_meas, X):

    """
    Given a solution of the inertial params regression problem, return the regression error associated with the measurements.

    Args:
        regressor: regressor matrix (n_jnts x n_samples x X_dim)
        tau_meas: measured tau
        X: regression solution vector
    
    """

    n_samples = len(tau_meas[0,:]) # number of samples per joint 
    n_jnts = len(tau_meas[:,0])

    error = np.zeros((n_jnts, n_samples))
    mean_abs_error = np.zeros((n_jnts, 1)).flatten()

    for i in range(n_jnts):
        error[i,:] = np.dot(regressors[i, :, :], X) - tau_meas[i, :]
        mean_abs_error[i] = np.mean(np.abs(error[i,:]))

    return error, mean_abs_error

