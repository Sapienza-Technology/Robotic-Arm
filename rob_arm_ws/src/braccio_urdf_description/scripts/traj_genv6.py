#!/usr/bin/env python

import math
from time import sleep
from turtle import position
import scipy
import swift
import roboticstoolbox as rtb
from roboticstoolbox import ETS, ERobot
from trajectory import Trajectory
import spatialmath as sm
from spatialmath import base
import numpy as np
import qpsolvers as qp
from sasa_functions import *
import numpy as np
import math as m

from decimal import *
getcontext().prec = 30
Decimal(1) / Decimal(7)

def premade_traj(no, steps, starting_q:np.array):
    if(no=="paletta"):
        task=False
        c_all = [np.array([0.5, 0., 0.3]), np.array([-0.349000, 0, 0.2210000]), np.array([-0.42, 0, 0.15]), np.array([-0.41, 0, 0.22]), np.array([0.4, 0., 0.2])]
        R_all = [Ry(-np.pi), Ry(3*pi/4), Ry(-4*pi/5), Ry(-4*pi/5), Ry(-np.pi)]
        elbow_all = [True, True, True, True, True]
    elif(no=="pirulo"):
        task=True
        c_all = [np.array([-0.40, 0.140, 0.315]), np.array([-0.493, 0.140, 0.315]), np.array([-0.513, 0.140, 0.169]), np.array([-0.493, 0.140, 0.32]), np.array([-0.40, 0.140, 0.32]), np.array([0.4, 0, 0.3]), np.array([0.4, 0, -0.10])]
        R_all = [Ry(-np.pi), Ry(-np.pi), Ry(-np.pi), Ry(-np.pi), Ry(-np.pi), Ry(-np.pi), Ry(-np.pi)]
        elbow_all = [True, True, True, True, True, True, True]
    elif(no=="inPose"):
        task = False
        c_all = [np.array([0.4, 0.1, 0.3])]
        R_all = [Ry(np.pi/2)]
        elbow_all = [True]
    elif(no == "prova"):
        task = True
        c_all = [np.array([-0.5, 0.15, 0.3])]
        R_all = [Ry(np.pi/2)]
        elbow_all = [True]
    else:
        print("Give as input pirulo or paletta or prova")
    return era_traj(task, starting_q, c_all, R_all, elbow_all, steps)

"""
def premade_traj_time(no, time_array:np.array, starting_q:np.array):
    if(no=="paletta"):
        task=False
        c_all = [np.array([0.5, 0., 0.3]), np.array([-0.349000, 0, 0.2210000]), np.array([-0.42, 0, 0.15]), np.array([-0.41, 0, 0.22]), np.array([0.4, 0., 0.2])]
        R_all = [Ry(-np.pi), Ry(3*pi/4), Ry(-4*pi/5), Ry(-4*pi/5), Ry(-np.pi)]
        elbow_all = [True, True, True, True, False]
    elif(no=="pirulo"):
        task=True
        c_all = [np.array([-0.40, 0.175, 0.31]), np.array([-0.50, 0.175, 0.31]), np.array([-0.50, 0.175, 0.165]), np.array([-0.50, 0.175, 0.31]), np.array([-0.40, 0.175, 0.31]), np.array([0.3, 0, 0.3])]
        R_all = [Ry(-np.pi), Ry(-np.pi), Ry(-np.pi), Ry(-np.pi), Ry(-np.pi), Ry(-np.pi)]
        elbow_all = [True, True, True, True, True, True]
    elif(no=="vuoto"):
        task=True
        c_all = [np.array([0.3, 0.2, 0.3]), np.array([-0.3, 0, 0.221]), np.array([-0.42, 0, 0.15]),np.array([0.3, 0.2, 0.3])]
        R_all = [Ry(np.pi)]
        elbow_all = [True, True, True]
    elif(no == "prova"):
        task = True
        c_all = [np.array([0.5, 0., 0.3])]
        R_all = [Ry(np.pi/2)]
        elbow_all = [True]
    else:
        print("Give as input pirulo or paletta or prova")
    return era_traj_time(task, starting_q, c_all, R_all, elbow_all, time_array)
"""

def era_traj(task:bool, present_joint_vals:np.array, position_all:np.array, rotation_all:np.matrix, elbow_all:np.array, steps:int):
    q_obj = np.zeros(6)
    #k_all = np.zeros(len(position_all))
    q_pres = present_joint_vals
    T_all = []
    for cont in range(len(position_all)):
        position = position_all[cont]
        rotation = rotation_all[cont]
        elbow = elbow_all[cont]
        q_obj = inv_kin_total_bis(position, rotation, task, elbow, q_obj[3:])
        print("Pose: ",q_obj)
        print("Cartesian position the real: ",dir_kin_total(q_obj,task)) 
        ########################################################################
        
        T1 = rtb.jtraj(q_pres, q_obj, steps)
        q_pres = q_obj
        # scale
        #T1, k = scale_traj(T1, vel_lim)
        #k = 1
        T_all.append(T1)
        #k_all[cont] = k
    return T_all

def era_traj_time(task:bool, present_joint_vals:np.array, position_all:np.array, rotation_all:np.matrix, elbow_all:np.array, time:np.array):
    q_obj = np.zeros(6)
    #k_all = np.zeros(len(position_all))
    q_pres = present_joint_vals
    T_all = []
    for cont in range(len(position_all)):
        position = position_all[cont]
        rotation = rotation_all[cont]
        elbow = elbow_all[cont]
        q_obj = inv_kin_total_bis(position, rotation, task, elbow, q_obj[3:])
        print("Pose: ",q_obj)
        print("Cartesian position the real: ",dir_kin_total(q_obj,task)) 
        ########################################################################
        
        time_array = np.array(list(range(time[cont])))
        T1 = rtb.jtraj(q_pres, q_obj, time_array)
        q_pres = q_obj
        #T1.plot(True)
        # scale
        #T1, k = scale_traj(T1, vel_lim)
        #k = 1
        T_all.append(T1)
        #k_all[cont] = k
    return T_all

"""
T_all = premade_traj("prova", 100, np.zeros(6))
#print(T_all[2].q)
q = T_all[0].q[99]
print(q)

print(dir_kin_total(np.array([q[0], q[1], q[2], q[3], q[4], q[5]]), True))
"""