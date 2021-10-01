import cv2
import numpy as np
import plotly.offline as py
import plotly.graph_objs as go
import matplotlib.pyplot as plt 

theta1begin = 35
theta1end = 145
theta2begin = 35
theta2end = 145

theta1begin_r = theta1begin*(np.pi/180)
theta1end_r = theta1end*(np.pi/180)
theta2begin_r = theta2begin*(np.pi/180)
theta2end_r = theta2end*(np.pi/180)

theta11 = np.linspace(theta1begin_r,theta1end_r)
theta22 = np.linspace(theta2begin_r, theta2end_r)
theta1, theta2 = np.meshgrid(theta11, theta22)

px1 = {}
py1 = {}
pz1 = {}

l1 = 3
l2 = 4

pxa = l1*np.cos(theta1) + l2*np.cos(theta2)
pya = l1*np.sin(theta1) + l2*np.sin(theta2)

print(pxa)
print(pya)

pxx = pxa
pyy = pya
pzz = pyy*0 #dummy zero points for z-axis, as it doesn't exist
trace1 = go.Surface(z=pzz, x=pxx, y=pyy,
                    colorscale='Reds', 
                    showscale=False, 
                    opacity=0.8,
                   )
data = [trace1]
layout = go.Layout(scene = dict(
                    xaxis = dict(title='X (mm)'),
                    yaxis = dict(title='Y (mm)'),
                    zaxis = dict(title='Z (mm)'),
                    yaxis_range=[-10,10],
                    xaxis_range=[-10,10],
                    zaxis_range=[-10,10]
                    ),
                  )
fig = go.Figure(data=data, layout=layout)
py.iplot(fig)








""" from plotly.offline import download_plotlyjs, init_notebook_mode, plot, iplot
init_notebook_mode(connected=True) # for offline mode in Jupyter Notebook use

import plotly.offline as py
import plotly.graph_objs as go
import numpy as np
from numpy import * # Not recommended, but we use to avoid rewriting the forward kinematic equations with prefix 'np'
theta1begin = 35
theta1end = 135
theta2begin = 35
theta2end = 135

theta1begin_r = theta1begin*(np.pi/180)
theta1end_r = theta1end*(np.pi/180)
theta2begin_r = theta2begin*(np.pi/180)
theta2end_r = theta2end*(np.pi/180)

theta11 = np.linspace(theta1begin_r,theta1end_r)
theta22 = np.linspace(theta2begin_r, theta2end_r)
theta1, theta2 = np.meshgrid(theta11, theta22)
l_range = [5] # we can use more than one value here

px1 = {}
py1 = {}
pz1 = {}
for i in l_range:
    l1 = i
    l2 = i - 4
    
    pxa = l1*cos(theta1) + l2*cos(theta1 + theta2)
    pya = l1*sin(theta1) + l2*sin(theta1 + theta2)
    
    px1['x{0}'.format(i)] = pxa
    py1['x{0}'.format(i)] = pya
pxx = px1['x5']
pyy = py1['x5']
pzz = pyy*0 #dummy zero points for z-axis, as it doesn't exist
trace1 = go.Surface(z=pzz, x=pyy, y=pxx,
                    colorscale='Reds', 
                    showscale=False, 
                    opacity=0.7,
                   )
data = [trace1]
layout = go.Layout(scene = dict(
                    xaxis = dict(title='X (mm)'),
                    yaxis = dict(title='Y (mm)'),
                    zaxis = dict(title='Z (mm)'),
                    ),
                  )
fig = go.Figure(data=data, layout=layout)
py.iplot(fig) """