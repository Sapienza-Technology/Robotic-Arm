import roboticstoolbox as rtb
import numpy as np


puma = rtb.models.DH.Puma560()
tau = puma.rne(puma.qn, np.ones((6,))/5, np.zeros((6,)))
print(tau)

q = puma.qn
qd = np.zeros((6,))

puma.inertia(q)
puma.coriolis(q, qd)
puma.gravload(q)

qdd = puma.accel(q, tau, qd)
print(qdd)

#q = puma.fdyn(5, q, tau)

#print(q)

#tau = mycontrol(robot, t, q, qd, **args)