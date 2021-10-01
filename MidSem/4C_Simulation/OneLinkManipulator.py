#Videh Patel : videh.p@iitgn.ac.in
#19110192

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

#Crude Maniulator without any logic
#Takes only torque as input and calculates other properties using equations


class OneLinkManipulator:
    def __init__(self,l1 = 4, q1 = 0, q1_dot = 0, m1 = 1):
        self.l1 = l1        #Link1 length (m)
        self.m1 = m1        #Mass1 length (kg)

        self.state = np.array([q1, q1_dot]) #[(rad), (rad/s)]

        self.time = 0       #time elapsed since start (sec)

        self.energyMat = []

        self.dt = 1./100.

    def apply_torques(self, tau_1):   #Updates the state of system and return it

        q1, q1dot = self.state

        g = 9.81

        q1dot2 = (tau_1 - self.m1*g*self.l1*np.sin(q1))/self.m1/self.l1**2

        if (q1dot2 > 1e14):
            print('WARNING:: the acceleration values too large, system may not respond as calculated')
            print('________________________________________________________________________________')
            return None

        q1dot_n = q1dot + q1dot2*self.dt

        q1_n = q1 + (q1dot + q1dot_n)/2*self.dt

        self.state = np.array([q1_n, q1dot_n])
        self.time += self.dt

        self.torque_text1.set_text('torque1 (N-m): %.2f' % tau_1)
        self.energy_txt.set_text('energy (N-m): %.2f' % (self.m1*g*self.l1*np.cos(q1) + 1/6*self.m1*self.l1**2*q1dot_n**2))
        self.energyMat.append(self.m1*g*self.l1*np.cos(q1) + 1/6*self.m1*self.l1**2*q1dot_n**2)
        
        self.anim_update()
        return self.state

    def run(self):
        #Kick starts animation and dynamics
        plt.ion()
        self.fig, self.ax = plt.subplots()

        self.anim_init()

    def anim_init(self):
        #Initialises animation
        l1_end_position = self.l1*np.array((np.cos(self.state[0]), np.sin(self.state[0])))                        #End of link1

        self.l1_end_circle = plt.Circle(l1_end_position, radius=0.25, fc='#0077b6')
        self.l1_line = plt.Line2D((0, l1_end_position[0]), (0, l1_end_position[1]), color = "#0077b6" , lw=2.5)

        self.ax.axhline(-self.l1, alpha = 0)
        self.ax.axhline(self.l1 , alpha = 0)
        self.ax.axvline(self.l1, alpha = 0)
        self.ax.axvline(-self.l1, alpha = 0)

        self.ax.add_line(self.l1_line)
        self.ax.add_patch(self.l1_end_circle)

        self.time_text = self.ax.text(0.02, 0.95, 'time (seconds): 0.00', transform=self.ax.transAxes)
        self.torque_text1 = self.ax.text(0.02, 0.9, 'torque1 (N-m): 0.00', transform=self.ax.transAxes)

        self.angle1 = self.ax.text(0.02, 0.1, 'q1 (deg): 0.00', transform=self.ax.transAxes)
        self.energy_txt = self.ax.text(0.02, 0.05, 'energy (N-m): 0.00', transform=self.ax.transAxes)

        plt.axis('scaled')

    def anim_update(self):
        #Updates the animation as the current state: Should be called wherever state changes

        q1,*_ = self.state

        l1_end_position = self.l1*np.array((np.cos(q1), np.sin(q1)))

        self.l1_end_circle.set_center(l1_end_position)

        self.l1_line.set_data((0, l1_end_position[0]), (0, l1_end_position[1]))

        self.time_text.set_text('time (seconds): %.2f' % self.time)
        self.angle1.set_text('q1 (deg): %.2f' % (q1*180/np.pi))

        plt.pause(self.dt)

    def apply_constant_torques(self, torque_tup, force_tup = (0,0),time = 2):
        for i in range(int(time/self.dt)):
            self.apply_torques(torque_tup, force_tup)

##Setting functions:: Position setting ignores dynamics
    def set_position(self, angle):
        self.set_state(np.array([angle,0,0]))

    def set_state(self, state_array):
        #anim_updating_func
        self.state = state_array
        self.anim_update()

    def get_state(self):
        return self.state

    def get_ee_position(self):
        return (self.l1*np.cos(self.state[0]) ,self.l1*np.sin(self.state[0]))

    def reset(self, position = (0,0)):
        #Resets the RRM at a given position
        self.time = 0
        self.set_position(position)

aiko = OneLinkManipulator()
print('MSG:: OneLinkManipulator initialised, named aiko')
print('MSG:: link1 length = ' + str(aiko.l1) + ' m')
print('MSG:: link1 mass = ' + str(aiko.m1) + ' kg')
print('MSG:: time delta = ' + str(aiko.dt) + ' seconds')
aiko.run()
print('MSG:: Animation started')
print()
aiko.fig.canvas.manager.set_window_title('Your own OneLinkManipulator: AIKO')
