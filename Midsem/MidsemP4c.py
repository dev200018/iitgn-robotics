import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import math

# Parameters
l = 1
m = 1

g = 9.8
k = 1
# function that returns dy/dt


def model(y, t, m, l, g, u, k):
    theta = y[0]
    thetad = y[1]

    taud = -k*(theta + math.pi/2)
    taui = m * g * (l/2) * math.cos(theta)

    tau = taui + taud
    yd = [0, 0]
    yd[0] = thetad
    yd[1] = (tau - m * g * (l/2) * math.cos(theta))/(m * l * l)
    return yd


# initial condition
y0 = np.array([-math.pi/2 - 0.1, 0])
u = 0

# time points
t = np.linspace(0, 20, num=100)

# solve ODE
y = odeint(model, y0, t, args=(m, l, g, u, k))

print(y)

xf = []
yf = []
for i in range(len(t)):
    x1f = l*math.cos(y[i, 0])
    y1f = l*math.sin(y[i, 0])
    xf.append(xf)
    yf.append(yf)

# plot results

plt.plot(t, y[:, 0], color='g', label='Theta')
plt.xlabel('time')
plt.ylabel('y(t)')
plt.grid()
plt.show()
