import math
import numpy as np
import scipy
from sympy.core.evalf import N

n=int(input())
A=[0]*n
for i in range(n):
    print(i,"th matrix")
    di=int(input("di "))
    ai=int(input("ai "))
    alphai=float(input("angle1 "))
    alpha_i=math.radians(alphai)
    Thetai=float(input("angle2 "))
    Theta_i=math.radians(Thetai)
    A[i]=np.array([[np.cos(Theta_i),-(np.sin(Theta_i)*np.cos(alpha_i)),(np.sin(Theta_i)*np.sin(alpha_i)),ai*np.cos(Theta_i)],
        [np.sin(Theta_i),np.cos(Theta_i)*np.cos(alpha_i),-(np.cos(Theta_i)*np.sin(alpha_i)),ai*np.sin(Theta_i)],
        [0,np.sin(alpha_i),np.cos(alpha_i),di],
        [0,0,0,1]])
# b part
#end-effector position
Ident=np.identity(4)
z=[0]*(n+1)
z[0]=np.array([[0],
               [0],
               [1]])

o=[0]*(n+1)
o[0]=np.array([0, 0, 0])
for i in range(n):
    Ident=Ident@A[i]
    z[i+1]=Ident[0:3, 0:3]@z[0]
    o[i+1]=Ident[0:3,3].T
# print("The end-effector position: ", Ident[0:3,3].T)
#a-part
#For the Manipulator Jacobian
#IF the joint q is R or P
#We give it by
Jacob=[0]*n
for i in range(n):
    q=input("The type of joint: ")
    if q=="P":
        Jacob[i]=np.concatenate((z[i],np.array([[0,0,0]]).T),axis=0)
    elif q=="R":
        z_cross=np.cross(z[i].T,(o[n]-o[i]).T)
        Jacob[i]=np.concatenate((z_cross.T,z[i]),axis=0)
Jacobian=Jacob[0]
for i in range(1,n):
    Jacobian=np.concatenate((Jacobian,Jacob[i]),axis=1)
print("The complete Jacobian Manipulator: ", Jacobian)

end_effec_velo=list(map(float, input("Enter Velocity of end-effector: ").split()))
Velo=np.array(end_effec_velo).reshape(6,1)
jacobian_inv = np.linalg.pinv(Jacobian)

print("Pseudo Inverse Jacobian: ",jacobian_inv)
#Joint velocities
joint_velo = np.matmul(jacobian_inv, Velo)
print("Joint Velocities: ",joint_velo)

