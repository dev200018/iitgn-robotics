#imports

import xlrd
from matplotlib import animation,pyplot as plt
import numpy as np

#Animation
fig,ax=plt.subplots()


l1=39
l2=40
x0=0
y0=35

def updatePlot(i,Q1,Q2,X,Y):
    q1=Q1[i]
    q2=Q2[i]

    x1=x0+l1*np.cos(q1)
    y1=y0-(l1*np.sin(q1)) # negative because the hip is upwards and we are taking y in negative direction with respect to it

    x2=x1+l2*np.cos(q2+q1)
    y2=y1-(l2*np.sin(q2+q1))
    
    ax.clear() # clear figure before each plot
    plt.xlabel(" Global X axis")
    plt.ylabel("Global Y axis")
    plt.title("X, Y plot with hip as (0,35)")
    # set axis limits. Without this the limits will be autoadjusted which will make it difficult to understand.
    ax.set_xlim([-5, 60])
    ax.set_ylim([-5, 20+y0])
    #ax.invert_yaxis()
    # ax.plot( [y0,y1],[x0,x1], 'b-o')
    # ax.plot([y1,y2],[x1,x2],'y-o')
    ax.plot([x0,x1], [y0,y1], 'b-o')
    ax.plot([x1,x2],[y1,y2],'y-o')

    ax.plot(X,Y, 'k--',linewidth=1)
    #ax.plot(Y,X, 'k--',linewidth=1) #Plotting trajectory line

if __name__ == "__main__":
    workbook = xlrd.open_workbook(r"Gait_DATA.xlsx")
    sheet = workbook.sheet_by_index(0)

    X = sheet.col_values(0, start_rowx=1, end_rowx=None)
    Y = sheet.col_values(1, start_rowx=1, end_rowx=None)
    
    #Ploting and saving
    ax.plot(X,Y)
    #ax.invert_yaxis()
    plt.xlabel(" Global X axis")
    plt.ylabel("Global Y axis")
    plt.title("X, Y plot with hip as (0,35)")
    plt.savefig("Gaiy_cycle_plot.png")
    ax.clear()

    Q1=[]
    Q2=[]

    for x,y in zip(X,Y):
        y=y0-y #because the hip is upwards and we are taking y in negative direction with respect to it
        q2=np.arccos((x**2 + y**2 -l1**2 -l2**2)/(2*l1*l2))
        q1=np.arctan2(y,x) - np.arctan2(l2*np.sin(q2),l1+l2*np.cos(q2))
        Q1.append(q1)
        Q2.append(q2)

    anim=animation.FuncAnimation(fig,updatePlot,frames=len(X),interval=60,fargs=[Q1,Q2,X,Y,])

    anim.save('18110131_q3c.mp4')