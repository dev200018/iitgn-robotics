#Videh Patel : videh.p@iitgn.ac.in
#19110192

from OneLinkManipulator import*
from matplotlib.backend_bases import MouseButton


class OneLinkManipulator_Controller:
    #Calculates and applies complex sets of torques to produce torsion spring output
    def __init__(self, OLMmodel):
        self.Calliberate(OLMmodel)
        self.dt = OLMmodel.dt
        self.m1 = OLMmodel.m1
        self.l1 = OLMmodel.l1
        self.OLMAppliesTorques = OLMmodel.apply_torques
        self.fig = OLMmodel.fig
        self.ax = OLMmodel.ax
        self.OLMmodel = OLMmodel
        self.mchaji = 0
        self.spring_acting = False
        self.spring_k = None
        self.spring_pt = (None,None)

    def Applies_Torques(self, torq_tup):
        self.state = self.OLMAppliesTorques(torq_tup)
        self.time += self.dt

    def Calliberate(self, OLMmodel):
        self.state = OLMmodel.get_state()
        self.time = OLMmodel.time

    def Torsion_Spring_Act(self, kappa, q0): #Calculates the torques and applies for 10 sec to make it act like a torsion spring 
        #torsion constant and mean angle taken as input
        cont = True
        g = 9.81

        for i in range(int(10/self.dt)):
            q1,*_ = self.state
            tau_ex = -kappa*(q1-q0) + self.m1*g*self.l1*np.sin(q1)
            self.Applies_Torques(tau_ex)

    def Get_EE_Position(self):
        return (self.l1*np.cos(self.state[0]) + self.l2*np.cos(self.state[1]),self.l1*np.sin(self.state[0]) + self.l2*np.sin(self.state[1]))


aikoCon = OneLinkManipulator_Controller(aiko)
print('MSG:: OneLinkManipulator_Controller initialised, named aikoCon')
