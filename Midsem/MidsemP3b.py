import csv
import numpy as np
import matplotlib.pyplot as plt


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

# Plotting data
plt.plot(x, y)

# naming the x axis
plt.xlabel('x (cm)')
# naming the y axis
plt.ylabel('y (cm)')

# giving a title to my graph
plt.title('ankle coordinates over the entire gait cycle')

plt.show()