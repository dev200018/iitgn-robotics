import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

Gait_data = pd.read_csv (r'F:\Robotics course\Mid Sem\Gait_DATA.csv')
#print(Gait_data)

X = np.array(pd.DataFrame(Gait_data,columns=['X (cm)']))
Y = np.array(pd.DataFrame(Gait_data,columns=['Y (cm)']))

plt.plot(X,Y)
plt.title('Y vs X')
plt.xlabel('X')
plt.ylabel("Y")
plt.show()

