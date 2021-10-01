from sympy import symbols, Eq, solve
import math
import matplotlib.pyplot as plt
import cv2
## assuming all input of physical parameters suitable for reach end-effector position 
m1 = 1
m2 = 1
l1 = 3
l2 = 4
I1 = 1
I2 = 1


#making wall 

wallpoint1 = [0,8]
wallpoint2 = [6,0]  

wallx_values =  [wallpoint1[0],wallpoint2[0]]
wally_values =  [wallpoint1[1],wallpoint2[1]]



plt.plot(wallx_values, wally_values, linewidth = 3)
plt.xlim(0,8)
plt.ylim(0,8)



##solving equation for dynamics

Fx = 30
Fy = 50

#initial EE positon
pinx = 4
piny = 0
#final EE position on wall
pfinx = 1.5
pfiny = 6

#making a linear way from initial and final position of EE

Px = [4]
Py = [0]

for t in range(1,100):
    p_x = (((-5*t)+789)/196)    
    p_y = ((12*t)/196)+(48/5)-((12*789)/(5*196)) 
    Px.append(p_x)
    Py.append(p_y)



# solving equations
q1l = []
q2l = []
    
for i in range (len(Px)):
    theta = math.acos((((Px[i])**2)+((Py[i])**2)-(l1**2)-(l2**2))/(2*l1*l2))
    q1 = math.atan(Py[i]/Px[i]) - math.atan(l2*math.sin(theta)/(l1 + l2*math.cos(theta)))
    q2 = q1 + theta
    q1l.append(q1)
    q2l.append(q2)



tow2 = l2*Fy*math.cos(q2l[99]) - Fx*l2*math.sin(q2l[99])
tow1 = l1*Fy*math.cos(q1l[99]) - Fx*l1*math.sin(q1l[99])

print('torques tow1 and tow2 anticlockwise are')
print(tow1)
print(tow2)


# coodinates od O1 
O1x = []
O1y = []
for i in range(len(Px)):
    o1x = l1*math.cos(q1l[i])
    o1y = l1*math.sin(q1l[i])
    O1x.append(o1x)
    O1y.append(o1y)

plt.ion()
plt.show()

""" plt.plot(wallx_values, wally_values, linewidth = 3) """


for i in range (len(Px)):

    o1pos = (O1x[i], O1y[i])
    ppos = (Px[i], Py[i])

    
    
    plt.clf()
    plt.xlim([-10,10])
    plt.ylim([-10,10])
    
    o0pos = (0, 0)
    plt.plot([o0pos[0], o1pos[0], ppos[0]], [o0pos[1], o1pos[1], ppos[1]], '-s')
    plt.plot(wallx_values, wally_values, linewidth = 3)
    for j in range(1,i):
        ppos = (Px[j], Py[j])
        plt.scatter(ppos[0], ppos[1],s= 1,c = "black")
    plt.pause(0.000001)



plt.ioff()
plt.show()

