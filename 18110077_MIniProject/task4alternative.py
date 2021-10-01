#program for Forward Kinematics of 2R Robotic Arm

import math
import matplotlib.pyplot as plt

# Length of link 1
l1 = 3
# Length of link 2
l2 = 4

#Defining angular range of link1 and link2

theta_start = 35*(math.pi/180)            #starting angke = 0 degrees 
theta_end = 145*(math.pi/180)             #end angle = degrees

#Defining number if intervales between theta_start & theta_end
theta_n = 20

# Angle traced in Radians by link 1 with respect to Horizontal x axis 
theta1 = []
# Angle traced in Radians by link 2 with respect to Horizontal x axis 
theta2 = []

# for loop ro calculate values of theta1 & theta2
for i in range(0, theta_n):
    angle_in_degrees = theta_start + i*(theta_end - theta_start)/(theta_n-1)    #calculating value in interval theta_n
    theta1.append(angle_in_degrees)   # adding values of theta in list
    theta2.append(angle_in_degrees)

#initial co-ordinates of link 1 at fixed pivot end in x and y is (0,0)
x0 = 0
y0 = 0

x1l = []
y1l = []
x2l = []
y2l = []

for i in range(0, theta_n):
    for j in range (0, theta_n):
        x1 = l1*math.cos(theta1[i])
        y1 = l1*math.sin(theta1[i])
        x2 = l1*math.cos(theta1[i])+l2*math.cos(theta2[j])
        y2 = l1*math.sin(theta1[i])+l2*math.sin(theta2[j])
        x1l.append(x1)
        y1l.append(y1)
        x2l.append(x2)
        y2l.append(y2)
        
for i in range (0, theta_n*theta_n):

    o1pos = (x1l[i], y1l[i])
    ppos = (x2l[i], y2l[i])
    o0pos = (0, 0)
    
    
    plt.clf()
    plt.xlim([-10,10])
    plt.ylim([-10,10])
    
    o0pos = (0, 0)
    plt.plot([o0pos[0], o1pos[0], ppos[0]], [o0pos[1], o1pos[1], ppos[1]], '-s')
    for j in range(1,i):
        ppos = (x2l[j], y2l[j])
        plt.scatter(ppos[0], ppos[1],s= 1,c = "black")
    plt.pause(0.0001)
""" for i in range (0, theta_n*theta_n):

    o1pos = (x1l[i], y1l[i])
    ppos = (x2l[i], y2l[i])
    o0pos = (0, 0)
    plt.xlim([-10,10])
    plt.ylim([-10,10])
    plt.scatter(ppos[0], ppos[1],s= 1,c = "black")
    
    plt.pause(0.000001) """

plt.ioff()
plt.show()