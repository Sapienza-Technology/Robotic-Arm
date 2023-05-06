import roboticstoolbox as rtb
import numpy as np


era = rtb.models.DH.Era()
tau = era.rne(era.qn, np.ones((6,))/5, np.zeros((6,)))
print(tau)

q = era.qn
qd = np.zeros((6,))

era.inertia(q)
era.coriolis(q, qd)
era.gravload(q)

qdd = era.accel(q, tau, qd)
print(qdd)

#q = era.fdyn(5, q, tau)

#print(q)

#tau = mycontrol(robot, t, q, qd, **args)

print(era)