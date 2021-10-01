#Answr for q4.
#Testing for Scara and stanford type
#Make sure that DH_jacobian_calculator.py is in same folder.

#Imports
import DH_jacobian_calculator as dh
import numpy as np
import os



def SCARA_DH(Corners,f):
    linkCount=3

    #Link Lengths
    l1=0.2
    l2=0.3
    l3=0.2
    print("SCARA : \n")
    f.write("\n\nSCARA:\n")
    for i in range(4):
        P0=Corners[i]
        x=float(P0[0,0])
        y=float(P0[1,0])
        z=float(P0[2,0])
        #Inverse kinematics
        q2=np.arccos((x**2+y**2-l2**2-l3**2)/(2*l3*l2))
        q1=np.arctan2(y,x)-np.arctan2((l3*np.sin(q2)),(l2+l3*np.cos(q2)))
        d3=l1-z   
        print("\nJoint Variables[q1,q2,d3] for corner "+chr(i+65)+" :")
        print([q1,q2,d3])
        f.write("\nJoint Variables[q1,q2,d3] for corner "+chr(i+65)+" :\n"+str([q1,q2,d3]))

        DH_param=np.zeros([linkCount,5],dtype=object)# First column link types, second for a, third for alpha , fourth for d and fifth for theta
        DH_param[0,0]='R'
        DH_param[0:1,1:5]=np.array([l2,0,l1,q1])
        DH_param[1,0]='R'
        DH_param[1:2,1:5]=np.array([l3,np.pi,0,q2])
        DH_param[2,0]='P'
        DH_param[2:3,1:5]=np.array([0,0,d3,0])

        P=np.array([[0],[0],[0],[1]])

        P0_calc=dh.End_Position(DH_param,P)
        print("\nEnd Position of corner "+chr(i+65)+" :")
        print(P0_calc)

        f.write("\nEnd Position of corner "+chr(i+65)+" :\n"+str(P0_calc))
        #Verification
        error=np.subtract(P0,P0_calc)
        if (np.abs(error) > 0.1).any():
            print("\nError in Position of corner "+chr(65+i)+" is more than 0.1")
            f.write("\nError in Position of corner "+chr(65+i)+" is more than 0.1\n")
        
        if (error < -0.0001).any() :
            print("Position of corner "+chr(65+i)+" is out of bounds\n")
            f.write("\nPosition of corner "+chr(65+i)+" is out of bounds\n\n")
    
        J0_P,P0_calc=dh.Jacobian_EndPoint_DH(DH_param,P)
        inv_J0_P=np.linalg.pinv(J0_P)
        X_dot=np.array([[0.01],[0.01],[0],[0],[0],[0]])#Velocities in x and y
        Q_dot=inv_J0_P@X_dot

        print("\n End point velocities: ")
        print(X_dot[0:3,0])
        f.write("\n End point velocities:\n "+str(X_dot[0:3,0]))

        print("\n Joint velocities: ")
        print(Q_dot)
        f.write("\n Joint velocities:\n "+str(Q_dot))

def Stanford_DH(Corners,f):
    
    linkCount=3

    #Link Lengths
    l1=0.2
    l2=0.25
    print("Stanford : \n")
    f.write("\n\nStanford : \n")
    for i in range(4):
        P0=Corners[i]
        x=float(P0[0,0])
        y=float(P0[1,0])
        z=float(P0[2,0])
        #Inverse kinematics
        r=np.sqrt(x**2 + y**2 - l2**2)
        q1= np.arctan2(y,x)+np.arctan2(l2,r)
        q2=np.arctan2(z-l1,r)
        d3=(z-l1)/np.sin(q2)
        print("\nJoint Variables[q1,q2,d3] for corner "+chr(i+65)+" :")
        print([q1,q2,d3])
        f.write("\nJoint Variables[q1,q2,d3] for corner "+chr(i+65)+" :\n"+str([q1,q2,d3]))

        DH_param=np.zeros([linkCount,5],dtype=object)# First column link types, second for a, third for alpha , fourth for d and fifth for theta
        DH_param[0,0]='R'
        DH_param[0:1,1:5]=np.array([0,np.pi/2,l1,q1])
        DH_param[1,0]='R'
        DH_param[1:2,1:5]=np.array([0,np.pi/2,l2,q2+np.pi/2])#q2+pi/2 so that x aligns with y axis and we can then rotate z on to prismatic side
        DH_param[2,0]='P'
        DH_param[2:3,1:5]=np.array([0,0,d3,0]) 

        P=np.array([[0],[0],[0],[1]])

        P0_calc=dh.End_Position(DH_param,P)
        print("\nEnd Position of corner "+chr(i+65)+" :")
        print(P0_calc)

        f.write("\nEnd Position of corner "+chr(i+65)+" :\n"+str(P0_calc))
        #Verification
        error=np.subtract(P0,P0_calc)
        if (np.abs(error) > 0.1).any():
            print("\nError in Position of corner "+chr(65+i)+" is more than 0.1")
            f.write("\nError in Position of corner "+chr(65+i)+" is more than 0.1\n")
        
        if (error < -0.0001).any() :
            print("Position of corner "+chr(65+i)+" is out of bounds\n")
            f.write("\nPosition of corner "+chr(65+i)+" is out of bounds\n\n")

        J0_P,P0_calc=dh.Jacobian_EndPoint_DH(DH_param,P)
        inv_J0_P=np.linalg.pinv(J0_P)
        X_dot=np.array([[0.01],[0.01],[0],[0],[0],[0]])#Velocities in x and y
        Q_dot=inv_J0_P@X_dot

        print("\n End point velocities: ")
        print(X_dot[0:3,0])
        f.write("\n End point velocities:\n "+str(X_dot[0:3,0]))

        print("\n Joint velocities: ")
        print(Q_dot)
        f.write("\n Joint velocities:\n "+str(Q_dot))

def PUMA_DH(Corners,f):
    
    linkCount=3

    #Link Lengths
    l1=0.15
    l2=0.3
    l3=0.2

    print("PUMA : \n")
    f.write("\n\nPUMA : \n")
    for i in range(4):
        P0=Corners[i]
        x=float(P0[0,0])
        y=float(P0[1,0])
        z=float(P0[2,0])
        #Inverse kinematics
        q1=np.arctan2(y,x)
        q3=-np.arccos((x**2+y**2+(z-l1)**2-l2**2-l3**2)/(2*l3*l2))
        q2=np.arctan2(z-l1,np.sqrt(x**2+y**2))-np.arctan2((l3*np.sin(q3)),(l2+l3*np.cos(q3)))
           
        print("\nJoint Variables[q1,q2,d3] for corner "+chr(i+65)+" :")
        print([q1,q2,q3])
        f.write("\nJoint Variables[q1,q2,d3] for corner "+chr(i+65)+" :\n"+str([q1,q2,q3]))

        DH_param=np.zeros([linkCount,5],dtype=object)# First column link types, second for a, third for alpha , fourth for d and fifth for theta
        DH_param[0,0]='R'
        DH_param[0:1,1:5]=np.array([0,np.pi/2,l1,q1])
        DH_param[1,0]='R'
        DH_param[1:2,1:5]=np.array([l2,0,0,q2])#q2+pi/2 so that x aligns with y axis and we can then rotate z on to prismatic side
        DH_param[2,0]='R'
        DH_param[2:3,1:5]=np.array([l3,0,0,q3]) 

        P=np.array([[0],[0],[0],[1]])

        P0_calc=dh.End_Position(DH_param,P)
        print("\nEnd Position of corner "+chr(i+65)+" :")
        print(P0_calc)
        f.write("\nEnd Position of corner "+chr(i+65)+" :\n"+str(P0_calc))

        #Verification
        error=np.subtract(P0,P0_calc)
        if (np.abs(error) > 0.1).any():
            print("\nError in Position of corner "+chr(65+i)+" is more than 0.1")
            f.write("\nError in Position of corner "+chr(65+i)+" is more than 0.1\n")
        
        if (error < -0.0001).any() :
            print("Position of corner "+chr(65+i)+" is out of bounds\n")
            f.write("\nPosition of corner "+chr(65+i)+" is out of bounds\n\n")

        J0_P,P0_calc=dh.Jacobian_EndPoint_DH(DH_param,P)
        inv_J0_P=np.linalg.pinv(J0_P)
        X_dot=np.array([[0.01],[0.01],[0],[0],[0],[0]])#Velocities in x and y
        Q_dot=inv_J0_P@X_dot

        print("\n End point velocities: ")
        print(X_dot[0:3,0])
        f.write("\n End point velocities:\n "+str(X_dot[0:3,0]))

        print("\n Joint velocities: ")
        print(Q_dot)
        f.write("\n Joint velocities:\n "+str(Q_dot))

#Main function calling above 3 parts
if __name__=="__main__":
    userInput=0
    
    #Corners
    A=np.array([[0.4],[0.06],[0.1],[1]])
    B=np.array([[0.4],[0.01],[0.1],[1]])
    C=np.array([[0.35],[0.01],[0.1],[1]])
    D=np.array([[0.35],[0.06],[0.1],[1]])
    corners=[A,B,C,D]
    f=open('q1d_output.txt','w')
    f.write(" Corners [A,B,C,D] : ")
    f.write(str(corners))
    f.write("\n\n")

    while not(userInput == 4):
        print("Choose Manipulator : \n1. SCARA\n2. Stanford\n3. Puma\n4. Exit\nEnter index for respective result: ")
        userInput=int(str(input()))
        if userInput==1:
            SCARA_DH(corners,f)           
        elif userInput==2:
            Stanford_DH(corners,f)
        elif userInput==3:
            PUMA_DH(corners,f)
        elif userInput==4:
            f.close()
            os._exit(0)
        else:
            print("Invalid Input. Enter again\n")
    os._exit(0)
