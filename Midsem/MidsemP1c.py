import numpy as np
import matplotlib.pyplot as plt
import math

# Four end points of Manifold
# E1 = [45, 7.5, 10], E2 = [45, -7.5, 10]
# E3 = [25, 7.5, 10], E4 = [25, -7.5, 10]

Type = input("Enter Robot Type (1-->PUMA, 2-->Stanford, 3-->SCARA): ")
Type = float(Type)

A = [40, 6, 10]
B = [40, 1, 10]
C = [35, 1, 10]
D = [35, 6 , 10]


# Check for PUMA RRR Robot
# Inverse Kinematics for PUMA RRR, From Github Repo


def ikpuma(x, y, z, d1, d2, d3):
    # using formulae from the textbook
    theta1 = math.atan(y/x)

    theta3 = math.acos((x**2 + y**2 + (z-d1)**2 - d2**2 - d3**2)/(2*d2*d3))
    theta2 = math.atan(z/((x**2 + y**2)**0.5) - math.atan((d3*math.sin(theta3))/(d2 + d3*math.cos(theta3))))

    print("First Solution PUMA: \n", "Theta1 = ", theta1, "\n Theta2 =", theta2, "\n Extension: ", theta3, "\n")
    #print("Second Solution PUMA: \n", "Theta1 = ", np.pi + theta1, "\n Theta2 =", np.pi - theta2, "\n Extension:", theta3)
    return theta1, theta2, theta3


# Check for Stanford Robot
# Inverse Kinematics for Stanford, From Github Repo


def ikstanford(endeffector_position, lengthsoflinks):
    theta1 = np.arctan(endeffector_position[1]/endeffector_position[0])
    r = np.sqrt(endeffector_position[0]**2 + endeffector_position[1]**2)
    s = endeffector_position[2] - lengthsoflinks[0]
    theta2 = np.arctan(s/r)
    d3 = np.sqrt(r**2 + s**2) - lengthsoflinks[1]
    print("First Solution: \n", "Theta1 = ", theta1, "\n Theta2 =", theta2,"\n Extension: ", d3, "\n")
    print("Second Solution: \n", "Theta1 = ", np.pi + theta1, "\n Theta2 =", np.pi - theta2,"\n Extension:", d3)
    return theta1, theta2, d3


# Check for SCARA Robot
# Inverse Kinematics for SCARA, From Github Repo


def ikscara(x, y, z, d1, d2):
    # using formulae from the textbook
    r = abs((x ** 2 + y ** 2 - d1 ** 2 - d2 ** 2) / (2 * d1 * d2))
    theta2 = np.arctan(np.sqrt(abs(1 - r ** 2)) / r)
    theta1 = np.arctan(y / x) - np.arctan(
        (d2 * np.sin(theta2)) / (d1 + d2 * np.cos(theta2)))
    d3 = -z
    print("First Solution SCARA: \n", "Theta1 = ", theta1, "\n Theta2 =", theta2, "\n Extension: ", d3, "\n")
    print("Second Solution SCARA: \n", "Theta1 = ", np.pi + theta1, "\n Theta2 =", np.pi - theta2, "\n Extension:", d3)
    return theta1, theta2, d3



def ikreturn(Type, A, B, C, D):
    if Type == 1:
        ParaA = ikpuma(A[0], A[1], A[2], 25, 25, 25)
        ParaB = ikpuma(B[0], B[1], B[2], 25, 25, 25)
        ParaC = ikpuma(C[0], C[1], C[2], 25, 25, 25)
        ParaD = ikpuma(D[0], D[1], D[2], 25, 25, 25)

    if Type == 3:
        ParaA = ikscara(A[0], A[1], A[2], 25, 25)
        ParaB = ikscara(B[0], B[1], B[2], 25, 25)
        ParaC = ikscara(C[0], C[1], C[2], 25, 25)
        ParaD = ikscara(D[0], D[1], D[2], 25, 25)

    if Type == 2:
        ParaA = ikscara(A, [25, 25])
        ParaB = ikscara(B, [25, 25])
        ParaC = ikscara(C, [25, 25])
        ParaD = ikscara(D, [25, 25])
    return  ParaA, ParaB, ParaC, ParaD


[ParaA, ParaB, ParaC, ParaD] = ikreturn(Type, A, B, C, D)

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
        T = forward_kinematics_homogenous_matrix(LinkParameters)
        coordinates = np.matmul(T, [[0],[0],[0],[1]])
    if Type == 2:
        LinkParameters = [[0, math.pi/2, 25, Para[0]], [0, math.pi/2, 25, Para[1] + math.pi/2], [0, 0, Para[2], 0]]
        T = forward_kinematics_homogenous_matrix(LinkParameters)
        coordinates = np.matmul(T, [[0], [0], [0], [1]])
    if Type == 3:
        LinkParameters = [[25, 0, 0, Para[0]], [25, math.pi, 0, Para[1]], [0, 0, Para[2], 0]]
        T = forward_kinematics_homogenous_matrix(LinkParameters)
        coordinates = np.matmul(T, [[0], [0], [0], [1]])
    return coordinates

a = fk(Type, ParaA)
b = fk(Type, ParaB)
c = fk(Type, ParaC)
d = fk(Type, ParaD)
print("Forward Kinematics results:")
print(a, b, c, d)