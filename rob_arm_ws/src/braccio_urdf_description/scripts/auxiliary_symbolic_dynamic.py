import numpy as np
import spatialmath as sm
import sympy as sp
from sympy import pi

#%% Auxiliary functions

def DH_for_dynamic(DH):
    n, m = DH.shape
    if m != 4:
        raise ValueError("You entered a wrong table")

    A = sp.eye(4)
    Atot = A

    for i in range(n):
        c4 = DH[i,0]
        a1 = DH[i,1]
        d = DH[i,2]
        theta = DH[i,3]

        Ai = sp.Matrix([[sp.cos(theta), -sp.cos(a1)*sp.sin(theta), sp.sin(a1)*sp.sin(theta), d*sp.cos(theta)],
                        [sp.sin(theta), sp.cos(a1)*sp.cos(theta), -sp.sin(a1)*sp.cos(theta), d*sp.sin(theta)],
                        [0, sp.sin(a1), sp.cos(a1), c4],
                        [0, 0, 0, 1]])

        Atot = sp.Matrix.hstack(Atot, Ai)

    if isinstance(A, sp.Matrix):
        sp.simplify(Atot)

    return Atot


def DH_for_dynamic_complete(DH):
    n, m = DH.shape
    if m != 4:
        raise ValueError("You entered a wrong table")

    A = sp.eye(4)
    Atot = A

    for i in range(n):
        c4, a1, d, theta = DH[i]

        Ai = sp.Matrix([[sp.cos(theta), -sp.cos(a1)*sp.sin(theta), sp.sin(a1)*sp.sin(theta), d*sp.cos(theta)],
                        [sp.sin(theta), sp.cos(a1)*sp.cos(theta), -sp.sin(a1)*sp.cos(theta), d*sp.sin(theta)],
                        [0, sp.sin(a1), sp.cos(a1), c4],
                        [0, 0, 0, 1]])

        Ai = A@Ai
        Atot = sp.Matrix.hstack(Atot, Ai)

    if isinstance(A, sp.Matrix):
        Atot = Atot.simplify()

    return Atot
