from sympy import symbols, Eq, solve
import math
import matplotlib.pyplot as plt
import cv2
## assuming all input of physical parameters suitable for reach end-effector position 
m1 = 1
m2 = 1
l1 = 1
l2 = 1
I1 = 1
I2 = 1  

## taking input of End-Effector point p(x,y) co-ordinates

# taking input of number of end-effector combination 
t_n = 200
px = []
py = []
for i in range(1,t_n):
    p_x = (i - 0.5)/100    
    p_y = (200 - i)/100 
    px.append(p_x)
    py.append(p_y)

# solving equations
q1l = []
q2l = []
    
for i in range (0, t_n-1):
    theta = math.acos(((px[i])**2+(py[i])**2-l1**2-l2**2)/2*l1*l2)
    q1 = math.atan(py[i]/px[i]) - math.atan(l2*math.sin(theta)/(l1 + l2*math.cos(theta)))
    q2 = q1 + theta
    q1l.append(q1)
    q2l.append(q2)


# coodinates od O1 
O1x = []
O1y = []
for i in range(0,t_n-1):
    o1x = l1*math.cos(q1l[i])
    o1y = l1*math.sin(q1l[i])
    O1x.append(o1x)
    O1y.append(o1y)

plt.ion()
plt.show()

for i in range (0, t_n-1):

    o1pos = (O1x[i], O1y[i])
    ppos = (px[i], py[i])

    
    
    plt.clf()
    plt.xlim([-5,5])
    plt.ylim([-5,5])

    o0pos = (0, 0)
    plt.plot([o0pos[0], o1pos[0], ppos[0]], [o0pos[1], o1pos[1], ppos[1]], '-s')
    for j in range(1,i):
        ppos = (px[j], py[j])
        plt.scatter(ppos[0], ppos[1],s= 1,c = "black")
    plt.pause(0.0001)

plt.ioff()
plt.show()
