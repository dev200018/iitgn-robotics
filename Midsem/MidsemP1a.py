import numpy as np
import matplotlib.pyplot as plt
import math

# Four end points of Manifold
# E1 = [45, 7.5, 10], E2 = [45, -7.5, 10]
# E3 = [25, 7.5, 10], E4 = [25, -7.5, 10]

# Check for PUMA RRR Robot
# Inverse Kinematics for PUMA RRR, From Github Repo


def ikpuma(x, y, z, d1, d2, d3):
    # using formulae from the textbook
    theta1 = math.atan(y/x)

    theta3 = math.acos((x**2 + y**2 + (z-d1)**2 - d2**2 - d3**2)/(2*d2*d3))
    theta2 = math.atan((z - d1)/((x**2 + y**2)**0.5)) - math.atan((d3*math.sin(theta3))/(d2 + d3*math.cos(theta3)))

    print("First Solution PUMA: \n", "Theta1 = ", theta1, "\n Theta2 =", theta2, "\n Theta: ", theta3, "\n")
    print("Second Solution PUMA: \n", "Theta1 = ", np.pi + theta1, "\n Theta2 =", np.pi - theta2, "\n Theta:", theta3)
    return [theta1, theta2, theta3]


ikpuma(45, 7.5, 10, 25, 25, 25)
ikpuma(45, -7.5, 10, 25, 25, 25)
ikpuma(25, 7.5, 10, 25, 25, 25)
ikpuma(25, -7.5, 10, 25, 25, 25)


# Check for Stanford Robot
# Inverse Kinematics for Stanford, From Github Repo


def ikstanford(x, y, z, d1, d2):
    theta1 = np.arctan(y/x) + np.arctan(d2/((x**2 + y**2 - d2**2)**0.5))
    theta2 = np.arctan((z-d1)/((x**2 + y**2 - d2**2)**0.5))
    d3 = (z - d1)/np.sin(theta2)
    print("First Solution: \n", "Theta1 = ", theta1, "\n Theta2 =", theta2,"\n Extension: ", d3, "\n")
    print("Second Solution: \n", "Theta1 = ", np.pi + theta1, "\n Theta2 =", np.pi - theta2,"\n Extension:", d3)
    return [theta1, theta2, d3]


ikstanford(45, 7.5, 10, 25, 25)
ikstanford(45, -7.5, 10, 25, 25)
ikstanford(25, 7.5, 10, 25, 25)
ikstanford(25, -7.5, 10, 25, 25)

# Check for SCARA Robot
# Inverse Kinematics for SCARA, From Github Repo


def ikscara(x, y, z, d1, d2, d3):
    # using formulae from the textbook
    theta2 = np.arccos((x ** 2 + y ** 2 - d2 ** 2 - d3 ** 2) / (2 * d3 * d2))
    theta1 = np.arctan2(y, x) - np.arctan2((d3 * np.sin(theta2)), (d2 + d3 * np.cos(theta2)))
    d3 = d1 - z

    print("First Solution SCARA: \n", "Theta1 = ", theta1, "\n Theta2 =", theta2, "\n Extension: ", d3, "\n")
    print("Second Solution SCARA: \n", "Theta1 = ", np.pi + theta1, "\n Theta2 =", np.pi - theta2, "\n Extension:", d3)
    return [theta1, theta2, d3]


ikscara(45, 7.5, 10, 0, 25, 25)
ikscara(45, -7.5, 10, 0, 25, 25)
ikscara(25, 7.5, 10, 0, 25, 25)
ikscara(25, -7.5, 10, 0, 25, 25)


Type  = 1
ParaA= ikpuma(25, -7.5, 10, 25, 25, 25)
def atransformation(LinkParameters):
    a = LinkParameters[0]
    d = LinkParameters[2]
    alpha = LinkParameters[1]
    theta = LinkParameters[3]
    A = np.array([[[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
                   [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
                   [0, np.sin(alpha), np.cos(alpha), d],
                   [0, 0, 0, 1]]])
    return (A)


def forward_kinematics_homogenous_matrix(DH):
    A = np.identity(4)
    n = np.shape(DH)[0]

    for i in range(n):
        Anext = atransformation(DH[i])
        A = np.matmul(A,Anext)

    return A

def fk(Type, Para):
    coordinates =[]
    if Type == 1:
        LinkParameters = [[0, math.pi/2, 25, Para[0]], [25, 0, 0, Para[1]], [25, 0, 0, Para[2]]]
        coordinates = forward_kinematics_homogenous_matrix(LinkParameters)
    if Type == 2:
        LinkParameters = [[0, math.pi/2, 25, Para[0]], [0, math.pi/2, 25, Para[1] + math.pi/2], [0, 0, Para[2], 0]]
        coordinates = forward_kinematics_homogenous_matrix(LinkParameters)
    if Type == 3:
        LinkParameters = [[25, 0, 0, Para[0]], [25, math.pi, 0, Para[1]], [0, 0, Para[2], 0]]
        coordinates = forward_kinematics_homogenous_matrix(LinkParameters)
    return coordinates


a = fk(Type, ParaA)
print(a)