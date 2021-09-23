import math
import numpy as np
from numpy.lib.function_base import extract
import scipy

#Given R_36 as a input we need to determine inverse kinematics of spherical wrist, that is find the euler angles
entries_R_36=list(map(float, input("Enter R_36 Matrix ").split()))
R_36=np.array(entries_R_36).reshape(3,3)
q4=(math.atan2(R_36[2,2],math.sqrt(1-(R_36[2,2])**2)))*(180/math.pi)
print("Angle q4: ",q4)
q5=(math.atan2(R_36[0,2],R_36[1,2]))*(180/math.pi)
print("Angle q5: ",q5)
q6=(math.atan2(-R_36[2,0],R_36[2,1]))*(180/math.pi)
print("Angle q6: ",q6)

