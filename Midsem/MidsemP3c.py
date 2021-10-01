import csv
import numpy as np
import math
import matplotlib.pyplot as plt

# Parameters
l1 = 45
l2 = 35

# Opening CSV File
file = open("Gait_DATA.csv")

# reading File Data
csvreader = csv.reader(file)

header = next(csvreader)
print(header)
x = []
y = []
for row in csvreader:
    x.append(row[0])
    y.append(row[1])
x = np.array(x)
y = np.array(y)

# converting data to float
x = list(np.float_(x))
y = list(np.float_(y))

file.close()

# Inverse Kinematics Function


def inverse_kinematics(x, y, l1, l2):
    y = 60 - y
    theta = np.arccos((x**2 + y**2 - l1**2 - l2**2)/(2*l1*l2))
    beta = math.atan(y/x)
    gamma = math.atan((l2*np.sin(theta))/(l1 + l2*np.cos(theta)))
    q1 = beta - gamma
    q2 = theta
    return q1, q2


Q1 = []
Q2 = []
for i in range(np.size(x)):
    Q = inverse_kinematics(x[i], y[i], l1, l2)
    Q1.append(Q[0])
    Q2.append(Q[1])

print(Q2)


Forward_x = []
Forward_y = []
for i in range(np.size(Q1)):
    Forward_x.append(l1 * math.cos(Q1[i]) + l2 * math.cos(Q1[i] + Q2[i]))
    Forward_y.append(60- l1 * math.sin(Q1[i]) - l2 * math.sin(Q1[i] + Q2[i]))

print(Forward_x)

plt.plot(Forward_x, Forward_y)

# naming the x axis
plt.xlabel('x (cm)')
# naming the y axis
plt.ylabel('y (cm)')

# giving a title to my graph
plt.title('ankle coordinates over the entire gait cycle')

plt.show()