

#Imports

import numpy as np
import scipy.integrate
import DH_jacobian_calculator as dh
from matplotlib import animation,pyplot as plt

#constants
m1=1
l1=10
g=9.81
k=100

#Initial States
q10=np.pi/4 +3*np.pi/2
q1_dot0=0

#Ground attached joint location
x0=0
y0=0

#Animation
fig,ax=plt.subplots()

def dynamicSystem(t,y):

    q1=y[0] -3*np.pi/2
    q1_dot=y[1]

    #Based on solution in pdf attached
    G=m1*g*(l1/2)*np.sin(q1)
    Tau=0 #Any external input torque
    Tau_i= - k*q1   
    q1_ddot=(Tau+Tau_i-G)/((1/3)*m1*l1**2)

    dydt=[q1_dot,q1_ddot] #q1_dot, q1_ddot, q2_dot, q2_ddot
    return dydt

def updatePlot(i,Q1):
    q1=Q1[i]
    P0=forward_kinematics(q1)
    x1=P0[0,0]
    y1=P0[1,0]
    
    ax.clear() # clear figure before each plot

    # set axis limits. Without this the limits will be autoadjusted which will make it difficult to understand.
    ax.set_xlim([-15, 15])
    ax.set_ylim([-15, 15])

    ax.plot([x0,x1], [y0,y1], 'b-o')

def forward_kinematics(q1):
    DH_param=np.zeros([1,5],dtype=object)# First column link types, second for a, third for alpha , fourth for d and fifth for theta
    DH_param[0,0]='R'
    DH_param[0:1,1:5]=np.array([l1,0,0,q1])
    P=np.array([[0],[0],[0],[1]])

    P0=dh.End_Position(DH_param,P)
    return P0

if __name__=="__main__":
    
    steps=1000
    time=np.linspace(0,steps/10,steps)

    solution=scipy.integrate.solve_ivp(dynamicSystem,[0, time[-1]],[q10,q1_dot0],t_eval=time)
    Q1=solution.y[0]
    Q1_dot=solution.y[1]

    anim=animation.FuncAnimation(fig,updatePlot,frames=steps,interval=60,fargs=[Q1,])

    anim.save('18110131_q4_k(100).mp4')

    total_energy=(1/6)*m1*(l1**2)*Q1_dot**2 + m1*g*(l1/2)*np.cos(Q1-3*np.pi/2)
    print(total_energy)