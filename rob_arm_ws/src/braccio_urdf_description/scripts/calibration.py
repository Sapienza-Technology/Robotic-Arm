#!/usr/bin/env python3

import numpy as np
import sympy

def calibration(n, error, theta_v):
    dh = sympy.MatrixSymbol('m', n, 4)
    dh = sympy.Matrix(dh)
    omogenea=directKin(dh)
    angles = [sympy.atan2(omogenea[2][1], omogenea[2][2]),
             sympy.atan2(-omogenea[2][0], sympy.sqrt(omogenea[2][1]**2 + omogenea[2][2]**2)),
             sympy.atan2(omogenea[1][0], omogenea[0][0])]
    
    pos = omogenea[0:3, 3]
    theta = sympy.flatten(dh)
    J = [[sympy.diff(pos, theta)],
         [sympy.diff(angles, theta)]]
    
    theta_res = theta_v
    tolerance = 10**(-4)
    for el in error:
        J_v = J.subs(theta, theta_res)
        dtheta = np.linalg.pinv(J_v) * el
        if dtheta < tolerance:
            return theta_res
        theta_res += dtheta
    return theta_res


def directKin(DH):
    n = DH.rows     # joint number

    out = sympy.eye(4)
    for i in range(1, n):
        out = sympy.simplify(out*DH1(DH[i,:]))

    return out

def DH1(row):
    alfa = row[0]
    a = row[1]
    d = row[2]
    theta = row[3]

    out = sympy.Matrix([[sympy.cos(theta), -sympy.cos(alfa)*sympy.sin(theta), sympy.sin(alfa)*sympy.sin(theta), a*sympy.cos(theta)], 
                  [sympy.sin(theta), sympy.cos(alfa)*sympy.cos(theta), -sympy.sin(alfa)*sympy.cos(theta), a*sympy.sin(theta)],
                  [0, sympy.sin(alfa), sympy.cos(alfa), d],
                  [0, 0, 0, 1]])

    return out



# per calibrare, crea la tabella DH, chiama la funzione directKin ancora simbolica
# e infine chiamare calibration