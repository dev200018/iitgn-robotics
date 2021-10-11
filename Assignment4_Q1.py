import math
import numpy as np
from numpy.lib.function_base import extract
import scipy

#In the inverse Kinematic Approach, we will be given by position vector O and Rotation Vector R.
#Enter the entries in a single Line row by row.
#taking entries for Roatational matrix R.
entries_R=list(map(float, input("Enter R Matrix ").split()))
R=np.array(entries_R).reshape(3,3)
#taking the position of end-effector. 
entries_d=list(map(float, input("Enter End-effector position ").split()))
d=np.array(entries_d).reshape(3,1)
#distance from Oc to end-effector position(d)
d6=float(input("Enter distance from wrist centre "))
#Since, stanford is a RRP manipulator, we need to find two angles q1 and q2
d1=float(input("Enter the distance between base and link1(d1) "))
d2=float(input("Enter the distance between link1 and link 2: "))
z=[0]*(3)
z[0]=np.array([[0],
               [0],
               [1]])
P=np.array([[0],[0],[1]])
P=d-d6*(R@z[0])
print("Position of wrist: ",P)
m1=abs((P[0,0]/P[1,0]))
q1=(math.atan(m1))*(180/math.pi)
print("Angle q1: ",q1)
D2=P[2,0]-d6
dxy=math.sqrt((P[0,0]**2)+(P[1,0]**2))
m2=(D2/dxy)
q2=(math.atan(m2))*(180/math.pi)
print("Angle q2: ",q2)
q3=math.sqrt((P[0,0]**2)+(P[1,0]**2)+(D2**2)) - d2
print("Length q3: ",q3)
Angle3=0
#finding R matrix for Stanford type robot
R_01=np.array([[np.cos(math.radians(q1)),-np.sin(math.radians(q1)),0],[np.sin(math.radians(q1)),np.cos(math.radians(q1)),0],[0,0,1]])
R_12=np.array([[np.cos(math.radians(q2)),-np.sin(math.radians(q2)),0],[np.sin(math.radians(q2)),np.cos(math.radians(q2)),0],[0,0,1]])
R_23=np.array([[np.cos(math.radians(Angle3)),-np.sin(math.radians(Angle3)),0],[np.sin(math.radians(Angle3)),np.cos(math.radians(Angle3)),0],[0,0,1]])
R_03=R_01@(R_12@R_23)
R_36=(np.linalg.inv(R_03))@R
q4=(math.atan2(R_36[2,2],math.sqrt(1-(R_36[2,2])**2)))*(180/math.pi)
print("Angle q4: ",q4)
q5=(math.atan2(R_36[0,2],R_36[1,2]))*(180/math.pi)
print("Angle q5: ",q5)
q6=(math.atan2(-R_36[2,0],R_36[2,1]))*(180/math.pi)
print("Angle q6: ",q6)



