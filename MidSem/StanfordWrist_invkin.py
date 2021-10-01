#@author: Videh Patel : videh.p@iitgn.ac.in: 19110192
#Stanford Inverse Kinematics
#Explanation attached in PDF submitted

import numpy as np

from R2EulerAngles import R2EulerAngles as R2EA
from DHMatrix2Homo_and_Jacob import DHMatrix2Homo_and_Jacob as DH2HnJ #Importing code of assignment3

def Stanford_invkin(T, dimensions):
    #Takes Transformation Matrix T and Stanford Manipulator (with shperical wrist) Dimensions as input and
    #returns joint params for a Stanford Manipulator to achieve the given position (Transformation Matrix)

    #dimensions is an array of form [l1, l2, l4, l5, l6]---> Link lengths
    ##Refer to pdf to look which link length corresponds to which
    R = T[:3,:3]
    d = T[:3,3]

    l1, l2, l4, l5, l6 = dimensions

    #Position of wrist centre
    p = d - (l6+l5)*(R@np.matrix([[0], [0], [1]])) 
    x,y,z = np.array(p.T)[0]

    #Inverse Position of wrist centre
    #First 3 params
    q1 = np.arctan2(y,x)
    q2 = np.arctan2(z-l1, np.sqrt(x**2 + y**2))
    d3 = np.sqrt(x**2 + y**2 + (z-l1)**2) - l2- l4
    if d3 < 0:
        raise('Point out of workspace')
        return None

    #Finding euler angles of wrist
    Rz_q1 = np.matrix([[np.cos(q1), -np.sin(q1), 0], [np.sin(q1), np.cos(q1), 0],[0,0,1]])
    Rx_90 =  np.matrix([[1,0,0],[0,0,-1],[0,1,0]])
    Rz_q2 = np.matrix([[np.cos(q2 + np.pi/2), -np.sin(q2 + np.pi/2), 0], [np.sin(q2 + np.pi/2), np.cos(q2 + np.pi/2), 0], [0,0,1]])
    
    R3_0 = Rz_q1@Rx_90@Rz_q2@Rx_90
    
    R6_3 = np.linalg.inv(R3_0)@R
    q4, q5, q6 = R2EA(R6_3)

    return q1, q2, d3, q4, q5, q6

if __name__ == '__main__':
    #Verifying with previous code
    dim = [5, 5, 2, 2, 2]
    # T_ = np.matrix([[0,0,1,11],[0,-1,0,0],[1,0,0,5], [0,0,0,1]]) #Another test case
    T_ = np.matrix([[1,0,0,11],[0,1,0,0],[0,0,1,5], [0,0,0,1]])
    l1, l2, l4, l5, l6 = dim
    q1, q2, d3, q4, q5, q6 = Stanford_invkin(T_ , dim)

    Dh = np.matrix([[q1, l1, 0, np.pi/2],[np.pi/2 + q2, 0, 0, np.pi/2 ],[0, l2 + d3 + l4, 0, 0],[q4,0,0,-np.pi/2],[q5,0,0,np.pi/2],[q6, l5+l6, 0,0]])

    H,J = DH2HnJ(Dh,[2])

    print('Homogenous Transformation used to calculate parameters::')
    print(T_)
    print('Calculated joint variables::')
    print(q1, q2, d3, q4, q5, q6)
    print('Homogenous Transformation used calculated with the parameters::')
    print(H)
    print('Rounding off and comparing element-wise::')
    print(np.round(H) == T_)
    