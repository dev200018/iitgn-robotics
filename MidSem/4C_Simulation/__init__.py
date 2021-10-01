from Controller import*
g = 9.81

aikoCon.Torsion_Spring_Act(20, np.pi/2)
fig, ax = plt.subplots()
plt.ioff()
ax.plot(np.linspace(0,10, 1000), aiko.energyMat)
ax.set_title('Energy of the system')
plt.xlabel('time (sec)')
plt.ylabel('energy (N-m)')
plt.show()