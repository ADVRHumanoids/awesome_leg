import casadi as cs
import numpy as np
from sympy import true

def quat2rot(Q):
    
    # first element = real component 

    q = cs.vertcat(Q[0], Q[1], Q[2], Q[3])

    q = q / cs.sqrt(q[0]* q[0] + q[1]* q[1] + q[2]* q[2] + q[3]* q[3]) # normalize input quat
    
    R = cs.vertcat(cs.horzcat(2 * (q[0] * q[0] + q[1] * q[1]) - 1,\
                   2 * (q[1] * q[2] - q[0] * q[3]),\
                   2 * (q[1] * q[3] + q[0] * q[2])),\
                   cs.horzcat(2 * (q[1] * q[2] + q[0] * q[3]),\
                   2 * (q[0] * q[0] + q[2] * q[2]) - 1,\
                   2 * (q[2] * q[3] - q[0] * q[1])),\
                   cs.horzcat(2 * (q[1] * q[3] - q[0] * q[2]),\
                   2 * (q[2] * q[3] + q[0] * q[1]),\
                   2 * (q[0] * q[0] + q[3] * q[3]) - 1))
              
    return R

def rot2quat(R, epsi = 0.0, lambd = 3.0):

    # convert matrix to quaternion representation

    # quaternion Q ={eta, Epsi}
    # where eta = cos(theta/2), with theta belogs [- PI, PI] 
    # Epsi = sin(theta/2) * r, where r is the axis of rotation
    
    eta = 1/2 * cs.sqrt(R[0, 0] + R[1, 1] + R[2, 2] + 1 + epsi)

    # smooth approximation of sign function
    epsi_1 = 1/2 * 2/ cs.pi * cs.arctan(lambd * R[2, 1] - R[1, 2]) * cs.sqrt(R[0, 0] - R[1, 1] - R[2, 2] + 1 + epsi) 
    epsi_2 = 1/2 * 2/ cs.pi * cs.arctan(lambd * R[0, 2] - R[2, 0]) * cs.sqrt(R[1, 1] - R[2, 2] - R[0, 0] + 1 + epsi) 
    epsi_3 = 1/2 * 2/ cs.pi * cs.arctan(lambd * R[1, 0] - R[0, 1]) * cs.sqrt(R[2, 2] - R[0, 0] - R[1, 1] + 1 + epsi)

    # epsi_1 = 1/2 * cs.sign(R[2, 1] - R[1, 2]) * cs.sqrt(R[0, 0] - R[1, 1] - R[2, 2] + 1 + epsi) 
    # epsi_2 = 1/2 * cs.sign(R[0, 2] - R[2, 0]) * cs.sqrt(R[1, 1] - R[2, 2] - R[0, 0] + 1 + epsi) 
    # epsi_3 = 1/2 * cs.sign(R[1, 0] - R[0, 1]) * cs.sqrt(R[2, 2] - R[0, 0] - R[1, 1] + 1 + epsi)
    
    Q = cs.vertcat(eta, epsi_1, epsi_2, epsi_3) # real part is conventionally assigned to the first component

    return Q

def Skew(vector):

    S = cs.vertcat( cs.horzcat(          0, - vector[2],   vector[1]), \
                    cs.horzcat(  vector[2],           0, - vector[0]), \
                    cs.horzcat(- vector[1] , vector[0] ,          0 ))

    
    return S

def rot_error(R_trgt, R_actual, epsi = 0.0):

    Q_trgt = rot2quat(R_trgt, epsi)
    Q_actual = rot2quat(R_actual, epsi)
    
    rot_err = Q_trgt[0] * Q_actual[1:4] - Q_actual[0] * Q_trgt[1:4] - \
              cs.mtimes(Skew(Q_actual[1:4]), Q_trgt[1:4])

    return rot_err

def rot_error2(R_trgt, R_actual, epsi = 0.0):

    R_err = R_trgt.T @ R_actual # orientation or actual frame w.r.t. target frame

    S = (R_err - R_err.T) / 2 # constant angular velocity necessary to bring frame actual to frame trgt ( * dt)

    r = cs.vertcat(S[2, 1], S[0, 2], S[1, 0])

    r_scaled = r / cs.sqrt(epsi + 1 + cs.trace(R_err))

    return r_scaled

def rot_error3(R_trgt, R_actual, epsi = 0.0):

    M_err = cs.DM_eye(3) - R_trgt.T @ R_actual

    # err = cs.trace(M_err)

    err = cs.vertcat(M_err[0, 0], M_err[1, 1], M_err[2, 2])

    return err

def compute_man_index(man_cost: list, n_int: int):

    man_measure = np.zeros((len(man_cost), 1)).flatten()

    for i in range(len(man_cost)): 

        man_measure[i] = 1 / np.sqrt(man_cost[i] / n_int) # --> discretized root mean squared joint velocities over the opt interval 

    return man_measure