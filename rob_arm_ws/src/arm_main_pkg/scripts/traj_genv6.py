#!/usr/bin/env python

import math
from time import sleep
from turtle import position
import scipy
import swift
import roboticstoolbox as rtb
from roboticstoolbox import ETS, ERobot
#from trajectory import Trajectory
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

def cartesian_traj(start_pos:np.array, end_pos:np.array, task:bool):
    '''
    This function generates a cartesian trajectory between two points
    
    Parameters
    ----------
    start_pos : np.array initial position
    end_pos : np.array final position
    task : bool True if the task is pirulation, False if the task is panel manipulation

    Returns
    -------
    X : np.array trajectory
    resolution : int number of steps
    '''

    distance = np.linalg.norm(end_pos-start_pos)
    resolution = int(distance/0.01)
    X = np.linspace(start_pos, end_pos, resolution)
    return X, resolution

def premade_traj(no, steps, starting_q:np.array):
    '''
    This function generates a preset trajectory for the robot arm
    Each trajectory is composed by a list of points, a list of rotations, a list of elbow states
    and a list of gripper states. The number of lists elements NEED to be coherent.

    Parameters
    ----------
    no : str name of the trajectory
    steps : int number of steps
    starting_q : np.array starting joint position of the robot arm

    Returns
    -------
    T_all : list of trajectories
    gripper_state : list of gripper states
    '''

    if(no=="paletta"):
        task=False
        c_all = [np.array([0.5, 0., 0.3]), np.array([-0.349000, 0, 0.2210000]), np.array([-0.42, 0, 0.15]), np.array([-0.41, 0, 0.22]), np.array([0.4, 0., 0.2])]
        R_all = [Ry(-np.pi), Ry(3*pi/4), Ry(-4*pi/5), Ry(-4*pi/5), Ry(-np.pi)]
        elbow_all = [True, True, True, True, False]
        gripper_state = [0.0, 0.0, 0.0, 0.0, 0.0]
    elif(no=="pirulo"):
        task=True
        c_all = [np.array([-0.40, 0.140, 0.315]), np.array([-0.493, 0.140, 0.315]), np.array([-0.513, 0.140, 0.169]), np.array([-0.493, 0.140, 0.32]), np.array([-0.40, 0.140, 0.32]), np.array([0.4, 0, 0.3]), np.array([0.4, 0, -0.10])]
        R_all = [Ry(-np.pi), Ry(-np.pi), Ry(-np.pi), Ry(-np.pi), Ry(-np.pi), Ry(-np.pi), Ry(-np.pi)]
        elbow_all = [True, True, True, True, True, True, True]
        gripper_state = [0.0, 0.0, 0.0, 0.0, 0.0]
    elif(no=="inPose"):
        task = False
        c_all = [np.array([0.55, 0, 0.25]),np.array([0.6, 0, 0.25])]
        R_all = [Ry(np.pi/2),Ry(np.pi/2)]
        elbow_all = [True,True]
        gripper_state = [0.0, 0.0]
    elif(no == "prova"):
        task = False
        c_all = [np.array([0.55, 0.0, 0.35]), np.array([0.6, 0.0, 0.3]),np.array([0.55, 0.0, 0.35]),np.array([0.55, 0.06, 0.35]),np.array([0.6, 0.06, 0.30]),np.array([0.45, 0.0, 0.40])]
        R_all = [Ry(np.pi/2), Ry(np.pi/2),Ry(np.pi/2),Ry(np.pi/2),Ry(np.pi/2),Ry(np.pi/2)]
        elbow_all = [True, True, True, True, True, True]
        gripper_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    elif(no == "cubo"):
        task = False
        X1, res1 = cartesian_traj(np.array([0.50, 0.0, 0.40]),np.array([0.50, -0.15, 0.40]),task)
        X2, res2 = cartesian_traj(np.array([0.50, -0.15, 0.40]),np.array([0.65, -0.15, 0.40]),task)
        X3, res3 = cartesian_traj(np.array([0.65, -0.15, 0.40]),np.array([0.65, 0.15, 0.40]),task)
        X4, res4 = cartesian_traj(np.array([0.65, 0.15, 0.40]),np.array([0.50, 0.15, 0.40]),task)
        X5, res5 = cartesian_traj(np.array([0.50, 0.15, 0.40]),np.array([0.50, 0.0, 0.40]),task)
        X6, res6 = cartesian_traj(np.array([0.50, 0.0, 0.40]),np.array([0.50, -0.15, 0.25]),task)
        X7, res7 = cartesian_traj(np.array([0.50, -0.15, 0.25]),np.array([0.65, -0.15, 0.25]),task)
        X8, res8 = cartesian_traj(np.array([0.65, -0.15, 0.25]),np.array([0.65, 0.15, 0.25]),task)
        X9, res9 = cartesian_traj(np.array([0.65, 0.15, 0.25]),np.array([0.50, 0.15, 0.25]),task)
        X10, res10 = cartesian_traj(np.array([0.50, 0.15, 0.25]),np.array([0.50, 0.0, 0.25]),task)

        c_all = np.row_stack((np.array([0.50, 0.0, 0.40]),X1, 
                            np.array([0.50, -0.15, 0.40]), X2, 
                            np.array([0.65, -0.15, 0.40]),X3,
                            np.array([0.65, 0.15, 0.40]),X4,
                            np.array([0.50, 0.15, 0.40]),X5,
                            np.array([0.50, 0.0, 0.40]),X6,
                            np.array([0.50, -0.15, 0.25]),X7,
                            np.array([0.65, -0.15, 0.25]),X8,
                            np.array([0.65, 0.15, 0.25]),X9,
                            np.array([0.50, 0.15, 0.25]),X10,
                            np.array([0.50, 0.0, 0.25])))
        R_all = [Ry(np.pi/2)]*(res1+res2+res3+res4+res5+res6+res7+res8+res9+res10+11)
        elbow_all = [True]*(res1+res2+res3+res4+res5+res6+res7+res8+res9+res10+11)
        gripper_state = [0.0]*(res1+res2+res3+res4+res5+res6+res7+res8+res9+res10+11)

    elif(no == "cerchio"):
        task = False
        X = []
        r = 0.10
        for kk in range(500):
            X.append(np.array([r*sin(2*pi/50*kk)+0.5, -r*cos(2*pi/50*kk), r*cos(2*pi/50*kk)*sin(pi/6)+0.5]))

        c_all = X
        R_all = [Ry(pi/6)]*(500)
        elbow_all = [True]*(500)
        gripper_state = [0.0]*(500)

    elif(no == "ballo"):
        task = False
        c_all = [np.array([0.50, 0, 0.20])]*500
        R_all = []
        for kk in range(500):
            R_all.append(Ry(np.pi+np.pi/6*cos(2*pi/50*kk))*Rx(np.pi/6*sin(2*pi/50*kk)))
        elbow_all = [True]*500
        gripper_state = [0.0]*500

    elif(no == "vuoto"):
        task = False
        start_pos =np.array([ 0.52229972, -0.04163132,  0.06202141])
        end_pos = np.array([ 0.57229972, 0.04163132,  0.06202141])
        distance = np.linalg.norm(end_pos-start_pos)
        resolution = int(distance/0.005)
        X = np.linspace(start_pos, end_pos, resolution)
        print("X: ", X)
        task = False
        c_all = np.row_stack((np.array([ 0.52229972, -0.04163132,  0.06202141]),X,np.array([ 0.57229972, 0.04163132,  0.06202141])))
        R_all = [Ry(np.pi/2)]*(resolution+2)
        elbow_all = [True]*(resolution+2)
        gripper_state = [0.0]*(resolution+2)

    elif(no == "vuoto2"):
        task = False
        c_all = [np.array([-0.20, -0.25, 0.07]), np.array([-0.20, -0.25, 0.04]),np.array([-0.20, -0.25, 0.07])]
        R_all = [Ry(np.pi),Ry(np.pi),Ry(np.pi)]
        elbow_all = [True, True, True]
        gripper_state = [0.0, 0.0, 0.0]

    else:
        print("Give as input pirulo or paletta or prova")
    return era_traj_vel(task, starting_q, c_all, R_all, elbow_all, steps), gripper_state


def era_traj_vel(task:bool, present_joint_vals:np.array, position_all:np.array, rotation_all:np.matrix, elbow_all:np.array, steps:int, q_gripper=0.0):
    '''
    This function generates a trajectory for the robot arm usinng jtraj from rtb

    Parameters
    ----------
    task : bool True if the task is pirulation, False if the task is panel manipulation
    present_joint_vals : np.array starting joint position of the robot arm
    position_all : np.array list of cartesian positions
    rotation_all : np.matrix list of rotations
    elbow_all : np.array list of elbow states
    steps : int number of steps
    q_gripper : float, optional
        DESCRIPTION. The default is 0.0.

    Returns
    -------
    T_all : list of trajectories    
    '''

    q_obj = np.zeros(6)
    #k_all = np.zeros(len(position_all))
    q_pres = present_joint_vals
    T_all = []
    for cont in range(len(position_all)):
        position = position_all[cont]
        rotation = rotation_all[cont]
        elbow = elbow_all[cont]
        q_obj = inv_kin_total_bis(position, rotation, task, elbow, q_obj, q_gripper)
        print("Pose: ",q_obj)
        print("Cartesian position the real: ",dir_kin_total(q_obj,task, q_gripper)) 
        q_const = (q_obj-q_pres)
        ########################################################################
        
        if cont == 0:
            T1 = rtb.jtraj(q_pres, q_obj, steps, np.zeros(6), q_const)
        elif cont == len(position_all)-1:
            T1 = rtb.jtraj(q_pres, q_obj, steps, q_const, np.zeros(6))
        else:
            T1 = rtb.jtraj(q_pres, q_obj, steps, q_const, q_const)
        q_pres = q_obj
        # scale
        #T1, k = scale_traj(T1, vel_lim)
        #k = 1
        T_all.append(T1)
        #k_all[cont] = k
    return T_all


### OLD VERSION ###############################################################################################################

def era_traj(task:bool, present_joint_vals:np.array, position_all:np.array, rotation_all:np.matrix, elbow_all:np.array, steps:int, q_gripper=0.0):
    q_obj = np.zeros(6)
    #k_all = np.zeros(len(position_all))
    q_pres = present_joint_vals
    T_all = []
    
    for cont in range(len(position_all)):
        position = position_all[cont]
        rotation = rotation_all[cont]
        elbow = elbow_all[cont]
        q_obj = inv_kin_total_bis(position, rotation, task, elbow, q_obj, q_gripper)
        print("Pose: ",q_obj)
        print("Cartesian position the real: ",dir_kin_total(q_obj,task, q_gripper)) 
        ########################################################################
        
        T1 = rtb.jtraj(q_pres, q_obj, steps)
        q_pres = q_obj
        # scale
        #T1, k = scale_traj(T1, vel_lim)
        #k = 1
        T_all.append(T1)
        #k_all[cont] = k
    return T_all

def era_traj_time(task:bool, present_joint_vals:np.array, position_all:np.array, rotation_all:np.matrix, elbow_all:np.array, time:np.array, q_gripper=0.0):
    q_obj = np.zeros(6)
    #k_all = np.zeros(len(position_all))
    q_pres = present_joint_vals
    T_all = []
    for cont in range(len(position_all)):
        position = position_all[cont]
        rotation = rotation_all[cont]
        elbow = elbow_all[cont]
        q_obj = inv_kin_total_bis(position, rotation, task, elbow, q_obj, q_gripper)
        print("Pose: ",q_obj)
        print("Cartesian position the real: ",dir_kin_total(q_obj,task, q_gripper)) 
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