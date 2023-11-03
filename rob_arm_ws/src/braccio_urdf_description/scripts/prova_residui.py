import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
import math as m
import sympy as sp

era = rtb.models.DH.Era(symbolic=True)

q = sp.symarray('q', 6)
qd = sp.symarray('qd', 6)
qdd = sp.symarray('qdd', 6)

#inertia = era.inertia(q)
#d_inertia = sp.diff(inertia, q)
era.symbolic = True
inertia = era.inertia(q)

print(inertia)