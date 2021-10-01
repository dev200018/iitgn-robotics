#Calculating jacobian for arbitrary manipulators using DH parameters.
#Import this file in your manipulator specific python file. 
#For using jacobian calculator function, the DH_parameters has to be an array of 5 elements with dtype=object
# First column of DH parameters link types, second for a, third for alpha , fourth for d and fifth for theta


#Imports:
import numpy as np
import sympy as sym #Using sympy so that symbols can also be used. #Use sym.N() to convert answers to float

#to make skew symmetric from vector
def skew(z):
    skew_z=np.zeros([3,3])
    skew_z[0,1]=-z[2,0]
    skew_z[1,0]=z[2,0]
    skew_z[0,2]=z[1,0]
    skew_z[2,0]=-z[1,0]
    skew_z[1,2]=-z[0,0]
    skew_z[2,1]=z[0,0]
    return skew_z

def DH_Transform(a,alpha,d,theta):
    #Transforming 1 DH frame to next
    #Theta is rotation about z axis,
    #d is displacement about z axis
    #a is displacement in x axis
    #alpha is rotation about z axis

    H=np.array([[sym.cos(theta)   ,-sym.sin(theta)*sym.cos(alpha)   , sym.sin(theta)*np.sin(alpha)   ,a*sym.cos(theta)],
                [sym.sin(theta)  , sym.cos(theta)*sym.cos(alpha)   , -sym.cos(theta)*np.sin(alpha)  ,a*sym.sin(theta)],
                [0              , sym.sin(alpha)                 , sym.cos(alpha)                 ,d              ],
                [0              ,0                              ,0                              ,1              ]])
    
    return H

def End_Position(DH_param,P):
    H_list=[np.eye(4)] #ith frame's H transform w.r.t. 0 frame is stored at ith position. list length is linkCount+1
    linkCount=DH_param.shape[0]

    for i in range(linkCount):
        H=DH_Transform(DH_param[i,1],DH_param[i,2],DH_param[i,3],DH_param[i,4])
        H_list.append(H_list[i]@H)
    P0=H_list[linkCount]@P #End point calculator
    return P0

def Jacobian_EndPoint_DH(DH_param,P):
    #To calculate Jacobian and end point w.r.t 0 frame 
    H_list=[np.eye(4)] #ith frame's H transform w.r.t. 0 frame is stored at ith position. list length is linkCount+1
    linkCount=DH_param.shape[0]

    for i in range(linkCount):
        H=DH_Transform(DH_param[i,1],DH_param[i,2],DH_param[i,3],DH_param[i,4])
        H_list.append(H_list[i]@H)

    O=np.array([[0],[0],[0],[1]]) #Represents origin in a frame. Used to convert origin of one particular frame to 0 frame perspective
    z=np.array([[0],[0],[1]]) # Z axis in a frame

    P0=H_list[linkCount]@P #End point calculator

    J0_P=np.zeros([6,linkCount]) #Jacobian matrix

    for i in range(linkCount):
        H=H_list[i]
        if(DH_param[i,0]=='P'):
            #Prismatic joint

            zi_1=H[0:3,0:3]@z   #Multiplying Z with rotation matrix to get z axis w.r.t 0 frame
            J0_P[0:3,i:i+1]=zi_1 #Linear velocity component due to revolute
        else:
            #Revolute Joint

            Oi_1=H@O
            zi_1=H[0:3,0:3]@z   #Multiplying Z of i-1 frame with rotation matrix to get z axis w.r.t 0 frame
            
            skew_zi_1=skew(zi_1) #Skew symmetric matrix for matrix multiplication
            
            J0_P[0:3,i:i+1]=np.matmul(skew_zi_1,np.subtract(P0,Oi_1)[0:3]) #Linear Velocity component due to revolute
            J0_P[3:6,i:i+1]=zi_1 #ANgular Velocity Component due to revolute

    return [J0_P,P0[0:3]]



if __name__=="__main__":

    #Test For above code.
    linkCount=2
    DH_param=np.zeros([linkCount,5],dtype=object)# First column link types, second for a, third for alpha , fourth for d and fifth for theta
    DH_param[0,0]='R'
    DH_param[1,0]='R'
    DH_param[0:1,1:5]=np.array([1.5,0,0,np.pi/4])
    DH_param[1:2,1:5]=np.array([1.5,0,0,np.pi/4])
    #q=pi/4,pi/4
    P=np.array([[0],[0],[0],[1]])

    J0_P,P0=Jacobian_EndPoint_DH(DH_param,P)
    print(J0_P)
    print(P0)

    q_dot=np.array([[1],[1]]) #each 1 rads per second

    #Velocity calculations
    v=J0_P[0:3,:]@q_dot
    w=J0_P[3:6,:]@q_dot

    print(v)
    print(w)

