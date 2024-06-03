"""
Code written based on servoj example from: https://github.com/davizinho5/RTDE_control_example
Edited by Nosa Edoimioya
10/10/2022
"""
import sys
sys.path.append('')
import logging
#import rtde.rtde as rtde
#import rtde.rtde_config as rtde_config
import scipy.io

import time
import copy
import numpy as np
import scipy.io as sio
#from control.matlab import *
import csv
from matplotlib import pyplot as plt
#from generate_sine_sweep_var_amp import TableTrajectoryVarying

np.set_printoptions(precision=7, suppress=True)

def get_inverse_kinematics(x, y, z, phi=0, theta=0, psi=np.pi/2):

    # rotation matrix
    R_xyz = np.array([[np.cos(phi)*np.cos(theta), -np.sin(phi)*np.cos(psi)+np.cos(phi)*np.sin(theta)*np.sin(psi), np.sin(phi)*np.sin(psi)+np.cos(phi)*np.sin(theta)*np.cos(psi)],
                    [np.sin(phi)*np.cos(theta), np.cos(phi)*np.cos(psi)+np.sin(phi)*np.sin(theta)*np.sin(psi), -np.cos(phi)*np.sin(psi)+np.sin(phi)*np.sin(theta)*np.cos(psi)],
                    [-np.sin(theta), np.cos(theta)*np.sin(psi), np.cos(theta)*np.cos(psi)]])

    tol = 10**(-3) # tolerance for computations
    q = np.zeros((6,))
    p_60 = np.array([[x],[y],[z]])
    H_60 = np.concatenate((R_xyz,p_60), axis=1)
    homogenous_concat = np.concatenate((np.zeros((1,3)),np.ones((1,1))), axis=1)
    H_60 = np.concatenate((H_60, homogenous_concat),axis=0)

    # q_1 -- find the origin of frame 5 by moving d_6 in the negative z direction
    unit_Z_60 = H_60[0:3,2]
    p_50 = p_60.flatten() - d6*unit_Z_60.flatten()
    q[0] = np.arctan2(p_50[1],p_50[0]) + np.arccos( d4 / (np.sqrt(p_50[0]**2 + p_50[1]**2)) ) + np.pi/2 # [rad]

    R_10 = np.array([[np.cos(q[0]), -np.sin(q[0]), 0],
                    [np.sin(q[0])*np.cos(alpha0), np.cos(q[0])*np.cos(alpha0), -np.sin(alpha0)],
                    [np.sin(q[0])*np.sin(alpha0), np.cos(q[0])*np.sin(alpha0), np.cos(alpha0)]])
    T_10 = np.concatenate((R_10,p_1), axis=1)
    T_10 = np.concatenate((T_10, homogenous_concat),axis=0)

    # q_5
    val = np.round((p_60[0]*np.sin(q[0]) - p_60[1]*np.cos(q[0])-d4) / d6, 4)
    q[4] = np.arccos( val )

    R_54 = np.array([[np.cos(q[4]), -np.sin(q[4]), 0],
                    [np.sin(q[4])*np.cos(alpha4), np.cos(q[4])*np.cos(alpha4), -np.sin(alpha4)],
                    [np.sin(q[4])*np.sin(alpha4), np.cos(q[4])*np.sin(alpha4), np.cos(alpha4)]])
    T_54 = np.concatenate((R_54,p_5), axis=1)
    T_54 = np.concatenate((T_54, homogenous_concat),axis=0)

    # q_6 
    unit_X_06 = np.matmul(np.transpose(R_xyz),np.array([[1],[0],[0]]))
    unit_Y_06 = np.matmul(np.transpose(R_xyz),np.array([[0],[1],[0]]))

    if np.abs(np.sin(q[4])) <= tol:
        q[5] = 0
    else:
        q[5] = np.arctan2((-unit_X_06[1]*np.sin(q[0])+unit_Y_06[1]*np.cos(q[0]))/np.sin(q[4]), \
            (unit_X_06[0]*np.sin(q[0])-unit_Y_06[0]*np.cos(q[0])/np.sin(q[4])) )
    
    R_65 = np.array([[np.cos(q[5]), -np.sin(q[5]), 0],
                    [np.sin(q[5])*np.cos(alpha5), np.cos(q[5])*np.cos(alpha5), -np.sin(alpha5)],
                    [np.sin(q[5])*np.sin(alpha5), np.cos(q[5])*np.sin(alpha5), np.cos(alpha5)]])
    T_65 = np.concatenate((R_65,p_6), axis=1)
    T_65 = np.concatenate((T_65, homogenous_concat),axis=0)

    # q_3
    p_40 = p_50 - d5*np.array([0,0,-1])
    p_41 = p_40 - d1*np.array([0,0,1])

    test_len = np.sqrt((p_41[0].flatten())**2 + (p_41[2].flatten())**2)
    if test_len >= np.abs(a2-a3)+tol and test_len <= np.abs(a2+a3)+tol:
        q[2] = np.arccos(((p_41[0]**2 + p_41[2]**2) - a2**2 - a3**2) / (2*a2*a3))
    else:
        # print an error message and break the loop
        raise Exception("The position is not in the robots reachable space. Exiting...")

    # q_2
    q[1] = np.arctan2(-p_41[2],-p_41[0]) + np.arcsin( a3*np.sin(q[2]) / test_len)

    #q_4
    R_21 = np.array([[np.cos(q[1]), -np.sin(q[1]), 0],
                    [np.sin(q[1])*np.cos(alpha1), np.cos(q[1])*np.cos(alpha1), -np.sin(alpha1)],
                    [np.sin(q[1])*np.sin(alpha1), np.cos(q[1])*np.sin(alpha1), np.cos(alpha1)]])
    T_21 = np.concatenate((R_21,p_2), axis=1)
    T_21 = np.concatenate((T_21, homogenous_concat),axis=0)

    R_32 = np.array([[np.cos(q[2]), -np.sin(q[2]), 0],
                    [np.sin(q[2])*np.cos(alpha2), np.cos(q[2])*np.cos(alpha2), -np.sin(alpha2)],
                    [np.sin(q[2])*np.sin(alpha2), np.cos(q[2])*np.sin(alpha2), np.cos(alpha2)]])
    T_32 = np.concatenate((R_32,p_3), axis=1)
    T_32 = np.concatenate((T_32, homogenous_concat),axis=0)

    T_30 = np.matmul(T_10,np.matmul(T_21,T_32))
    T_64 = np.matmul(T_54,T_65)

    T_43 = np.matmul(np.linalg.inv(T_30),np.matmul(H_60,np.linalg.inv(T_64)))

    X_43 = T_43[:,0]

    q[3] = np.arctan2(X_43[1],X_43[0])

    return np.round(q,4)

# paramaters
alpha0 = 0 # [rad]
alpha1 = np.pi/2
alpha2 = 0
alpha4 = np.pi/2
alpha5 = -np.pi/2

a0 = 0 # [m]
a1 = 0
a2 = -0.425
a3 = -0.3922
a4 = 0
a5 = 0

d1 = 0.1625 # length from base joint shoulder joint in the z-direction
#d1 = 0 # length from base joint shoulder joint in the z-direction
d2 = 0
d3 = 0
d4 = 0.1333 # [m]
d5 = 0.0997
d6 = 0.0996 # [m]

p_1 = np.array([[a0], [-np.sin(alpha0)*d1], [np.cos(alpha0)*d1]])
p_2 = np.array([[a1], [-np.sin(alpha1)*d2], [np.cos(alpha1)*d2]])
p_3 = np.array([[a2], [-np.sin(alpha2)*d3], [np.cos(alpha2)*d3]])
p_5 = np.array([[a4], [-np.sin(alpha4)*d5], [np.cos(alpha4)*d5]])
p_6 = np.array([[a5], [-np.sin(alpha5)*d6], [np.cos(alpha5)*d6]])

q_0 = 0 # base joint angle


# V = np.arange(0,(5*np.pi)/8,np.pi/8) # [0, pi/4, 3*pi/8, pi/2] V0.78 (2*pi/8), R0.56 stopped 10/13
# V = np.arange(0,np.pi,np.pi/4) # [0, pi/4, pi/2 3*pi/4]
# V = np.arange(0,(np.pi)/8,np.pi/8)

#R = np.arange(0.4,0.7,1/12)
#R = np.array([245.455,654.545,654.545,241.667])*1/1000
#V = np.array([45,45,78.75,5.625])*np.pi/180

mat = scipy.io.loadmat('Parameters_fixed_R.mat')
R        = np.array([mat['R_selec'][0,0],mat['R_selec'][1,0],mat['R_selec'][2,0],mat['R_selec'][3,0]]) # Note the values of R ara coming from the experiments
V        = np.array([0,0,0,0])*np.pi/180

V_end    = np.array([90,90,90,90])*np.pi/180

#for v in V:
#    print("-------Executing moveJ -----------\n")
#    for r in R:
for i in range(len(R)): 
       # coordinates of TCP in universal coordinates
        x = -R[i]*np.cos(V[i])*np.cos(q_0)
        y = -(d4+d6) - R[i]*np.cos(V[i])*np.sin(q_0)
        z = d1 + R[i]*np.sin(V[i])
        #print('initial x, y, z: ',x,', ',y,', ',z)
        print('Pose i = ',i,' R =' ,R[i]*1000, ' V = ',V[i]*180/np.pi)
      
        #q_init = get_inverse_kinematics(x,y,z)
        q_init = get_inverse_kinematics(x, y, z, phi=0, theta=V[i], psi=np.pi/2)
        print('Initial joint values q1 =',q_init[0],' q2 =',q_init[1],' q3 =',q_init[2],' q4 =',q_init[3],' q5 =',q_init[4],' q6 =',q_init[5])
        #        print(f"{q_init[0]:f}")
        # calculating the final joint values
        x = -R[i]*np.cos(V_end[i])*np.cos(q_0)
        y = -(d4+d6) - R[i]*np.cos(V_end[i])*np.sin(q_0)
        z = d1 + R[i]*np.sin(V_end[i])
        
        #print('initial Final x, y, z: ',x,', ',y,', ',z)
        q_init = get_inverse_kinematics(x, y, z, phi=0, theta=V_end[i], psi=np.pi/2)
        print('Final Pose i = ',i,' R =' ,R[i]*1000, ' V = ',V_end[i]*180/np.pi)
        print('Final joint values q1 =',q_init[0],' q2 =',q_init[1],' q3 =',q_init[2],' q4 =',q_init[3],' q5 =',q_init[4],' q6 =',q_init[5])
        print('\n')

    