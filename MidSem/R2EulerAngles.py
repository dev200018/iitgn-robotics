#@author: Videh Patel : videh.p@iitgn.ac.in: 19110192
#Converts calculates euler angles from rotation matrix by multiplying psuedoinverse of Jacobian
##Can also be used for inverse kinematics of Sheprical Wrist to achieve desired angular orientation 
#Followed the explaination in textbook

import numpy as np

def R2EulerAngles(R, positive_theta = True):
    #Takes a rotation matrix input and returns Euler's Angles (phi, theta, psi)
    #positive_theta directs the solution chosen for the final output
    #Takes phi = 0 whenever infinitely many solutions are possible
    if ((R[2,0] == 0) and (R[2,1] == 0)):
        phi = 0
        if R[2,2] == 1:
            theta = 0
            psi = np.arctan2(R[1,0],R[0,0])
        elif R[2,2] == -1:
            theta = np.pi if positive_theta else -np.pi
            psi = -np.arctan2(-R[1,0], -R[0,0])
        else:
            print('Invalid Rotation Matrix')
            return None
    else:
        if positive_theta:
            theta = np.arctan2(np.sqrt(1 - R[2,2]**2), R[2,2])
            phi = np.arctan2(R[1,2], R[0,2])
            psi = np.arctan2(R[2,1], -R[2,0])
        else: #Negative Theta
            theta = np.arctan2(-np.sqrt(1 - R[2,2]**2), R[2,2])
            phi = np.arctan2(-R[1,2], -R[0,2])
            psi = np.arctan2(-R[2,1], R[2,0])
    
    return phi, theta, psi

if __name__ == '__main__':
    R = np.matrix([[1,0,0],[0,0,-1],[0,-1,0]])

    print(R2EulerAngles(R))
