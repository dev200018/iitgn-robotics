#@author: Videh Patel : videh.p@iitgn.ac.in: 19110192
#SCARA Inverse Kinematics
#Code copied from Assignment 4
#Minor changes to incorporate changes in l0 (raised hieght of first revolute of scara)

import numpy as np

from R2EulerAngles import R2EulerAngles as R2EA
from DHMatrix2Homo_and_Jacob import DHMatrix2Homo_and_Jacob as DH2HnJ #Importing code of assignment3

def SCARA_invkin(T, dimensions):
    #Takes Transformation Matrix T and SCARA Manipulator (with shperical wrist) Dimensions as input and
    #returns joint params for a Stanford Manipulator to achieve the given position (Transformation Matrix)

    #dimensions is an array of form [l0, l1, l2, l4, l5, l6]---> Link lengths
    ##Refer to pdf to look which link length corresponds to which
    R = T[:3,:3]
    d = T[:3,3]

    l0, l1, l2, l4, l5, l6 = dimensions

    #Position of wrist centre
    p = d - (l6+l5)*(R@np.matrix([[0], [0], [1]])) 
    x,y,z = np.array(p.T)[0]

    #Inverse Position of wrist centre
    #First 3 params
    q2 = np.arccos((x*x + y*y - l1**2 - l2**2)/(2*l1*l2))
    q1 = np.arctan2(y,x) - np.arctan(l2*np.sin(q2)/(l1 + l2*np.cos(q2)))
    d3 = -z + l0 -l4
    if d3 < 0:
        raise('Point out of workspace')
        return None

    #Finding euler angles of wrist
    Rz_q1q2 = np.matrix([[np.cos(q1+q2), -np.sin(q1+q2), 0], [np.sin(q1+q2), np.cos(q1+q2), 0],[0,0,1]])
    Rx_180 =  np.matrix([[1,0,0],[0,-1,0],[0,0,-1]])
    
    R3_0 = Rz_q1q2@Rx_180
    
    R6_3 = np.linalg.inv(R3_0)@R
    q4, q5, q6 = R2EA(R6_3)

    return q1, q2, d3, q4, q5, q6


if __name__ == '__main__':
    #Verifying with previous code
    dim = [5, 5, 2, 2, 2]
    T_ = np.matrix([[1,0,0,10],[0,-1,0,0],[0,0,-1,-6], [0,0,0,1]])
    l1, l2, l4, l5, l6 = dim
    q1, q2, d3, q4, q5, q6 = SCARA_invkin(T_ , dim)

    Dh = np.matrix([[q1, 0, l1, 0],[q2, 0, l2, np.pi],[0, d3+l4, 0, 0],[q4, 0, 0, -np.pi/2], [q5, 0, 0, np.pi/2],[q6,l5+l6,0,0]])
    H,J = DH2HnJ(Dh,[3])

    print('Homogenous Transformation used to calculate parameters::')
    print(T_)
    print('Calculated joint variables::')
    print(q1, q2, d3, q4, q5, q6)
    print('Homogenous Transformation used calculated with the parameters::')
    print(H)
    print('Rounding off and comparing element-wise::')
    print(np.round(H) == T_)
    