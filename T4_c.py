# Name - Dev Patel
# Roll No. - 18110113

import numpy as np
import scipy.integrate
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


# define some constants

g = 9.81
l = 10
m = 10
q1 = 0
k1 = 100# Spring Constant
k2 = 100
x_0 = 10 # Mean position
y_0 = (l**2 - x_0**2)**0.5 # Mean position

# define initial states

x = 9.5
y = (l**2 - x**2)**0.5

q1 = np.arctan(y/x)
omega1 = 0




def Rmanipulator(t, y):
    '''
    function for the ODE solver - returns dy/dt of states
    '''
    omega1 = y[1]
    q1 = y[0]

    c = 0.1 # damping coefficient
    Torque_i = (k2 - k1)*(l**2)*np.cos(q1)*np.sin(q1) - k2*l*y_0*np.cos(q1) + k1*l*x_0*np.sin(q1)
    
    dydt = [omega1, (-Torque_i + m*g*l*np.cos(q1)/2)/(m*l*l/12) ]
    
    return dydt

    
initstate = [q1, omega1] # initial states
timestep = 0.1

time = np.linspace(0, 10, 1000) # define the time for which we want to solve


# solve the ODE
solution = solve_ivp(Rmanipulator, [0, time[-1]], initstate, t_eval= time) 


qs1 = solution.y[0]
omegas1 = solution.y[1]

# # now plotting.

# plt.ion()
# plt.show()

# for i in range(len(qs1)):
#     q1 = qs1[i]

#     # position of the links 
#     ballpos1 = (l*np.cos(q1),l*np.sin(q1))

#     plt.clf() # clear figure before each plot

#     # set axis limits. 
#     plt.xlim([-20, 20])
#     plt.ylim([-20, 20])

#     # plot the 2R Manipulator
#     hinge = (0, 0)
#     plt.plot([hinge[0], ballpos1[0]], [hinge[1], ballpos1[1]], '-o')

#     # pause so that the figure can be seen
#     plt.pause(0.0001)

# plt.ioff()
# plt.show()

# For plotting the plots of end effector position
X = []
Y = []
T = []

for i in range(len(qs1)):
    q1 = qs1[i]
    x = l*np.cos(q1)
    y = l*np.sin(q1)
    Te = m*g*l*np.sin(q1)/2 + m*l*l*omegas1[i]*omegas1[i]/6
    X.append(x)
    Y.append(y)
    T.append(Te)

plt.plot(time,Y,'r',time,X,'g')
plt.title('Position vs time')
plt.xlabel('Time')
plt.ylabel("position of end effector")
plt.legend(['y','x'])
plt.show()

plt.plot(time,T)
plt.title('Total Energy vs time')
plt.xlabel('Time')
plt.ylabel("Total Energy of system")
plt.show()