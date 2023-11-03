#!/usr/bin/env python

import math
from time import sleep
import time
from turtle import delay
import scipy
import swift
import roboticstoolbox as rtb
from roboticstoolbox import ETS, ERobot, angle_axis
from trajectory import Trajectory
import spatialmath as sm
from spatialmath import base
import numpy as np
#import qpsolvers as qp
from sasa_functions import *
import numpy as np
import math as m

## Useful rotations
def Rx(theta):
  return np.matrix([[ 1, 0           , 0           ],
                   [ 0, m.cos(theta),-m.sin(theta)],
                   [ 0, m.sin(theta), m.cos(theta)]])

def Ry(theta):
  return np.matrix([[ m.cos(theta), 0, m.sin(theta)],
                   [ 0           , 1, 0           ],
                   [-m.sin(theta), 0, m.cos(theta)]])

def Rz(theta):
  return np.matrix([[ m.cos(theta), -m.sin(theta), 0 ],
                   [ m.sin(theta), m.cos(theta) , 0 ],
                   [ 0           , 0            , 1 ]])



## Launch the simulator Swift
env = swift.Swift()
env.launch(realtime=True, headless=True, comms='websocket')

## define some useful variables
pi = np.pi
vel_lim = 1.5  # joint velocity limit 

## era model is imported and added to swift environment
era = rtb.ERobot.URDF("/home/toto/Documents/SASA/SASA/braccio_description/urdf/braccio.xacro")

print(era)
env.add(era)

#elbow rappresenta la configuazione "elbow up"==1 o "elbow down"==0
#phase rappresenza il task "phase pirul"==1 o "phase maintenance"==0

## To change #############################################################
# Meaningful trajectories ################################################
def premade_traj(no):
    if(no==1):
        task=False
        c_all = [np.array([0.4, 0, 0.2]), np.array([-0.349, -0.20, 0.221]), np.array([-0.42, 0, 0.15]), np.array([-0.41, 0, 0.22]), np.array([0.4, 0., 0.2])]
        R_all = [Ry(-np.pi), Ry(-4*pi/5), Ry(-4*pi/5), Ry(-4*pi/5), Ry(-np.pi)]
        elbow_all = [True, True, True, True, True]
    elif(no==2):
        task=False
        c_all = [np.array([0.4, 0., 0.2]), np.array([-0.48, -0.2, 0.221]), np.array([-0.5, 0, 0.15]), np.array([-0.5, 0, 0.22]), np.array([0.4, 0., 0.2])]
        R_all = [Ry(-np.pi), Ry(-4*pi/5), Ry(-4*pi/5), Ry(-4*pi/5), Ry(-np.pi)]
        elbow_all = [True, True, True, True, True]
    elif(no==3):
        task=False
        c_all = [np.array([0.3, 0.2, 0.3]), np.array([-0.349, 0, 0.221]), np.array([-0.42, 0, 0.15]),np.array([0.3, 0.2, 0.3])]
        R_all = [Ry(-np.pi), Ry(-4*pi/5), Ry(-np.pi)]
        elbow_all = [True, True, True]
    elif(no== 4):
        task=True
        c_all = [np.array([0.4, 0., 0.2]), np.array([-0.52, -0.2, 0.221]), np.array([-0.52, -0.2, 0.15]), np.array([-0.52, -0.2, 0.22]), np.array([0.4, 0., 0.2])]
        R_all = [Ry(-np.pi), Ry(-np.pi), Ry(-np.pi), Ry(-np.pi), Ry(-np.pi)]
        elbow_all = [True, True, True, True, True]
    elif(no == 5):
        task = False
        c_all = [np.array([0.4, 0.1, 0.3])]
        R_all = [Ry(np.pi/2)]
        elbow_all = [True]
    else:
        print("Give input of numbers from 1 to 3")
    return task, c_all, R_all, elbow_all


def premade_traj_time(no):
    if(no=="paletta"):
        task=False
        c_all = [np.array([0.5, 0., 0.3]), np.array([-0.349000, 0, 0.2210000]), np.array([-0.42, 0, 0.15]), np.array([-0.41, 0, 0.22]), np.array([0.4, 0., 0.2])]
        R_all = [Ry(np.pi), Ry(3*pi/4), Ry(-4*pi/5), Ry(-4*pi/5), Ry(np.pi)]
        elbow_all = [True, True, True, True, False]
    elif(no=="pirulo"):
        task=True
        c_all = [np.array([-0.47, -0.19, 0.25]), np.array([-0.51, -0.19, 0.25]), np.array([-0.51, -0.19, 0.15]), np.array([-0.51, -0.19, 0.25]), np.array([-0.47, -0.19, 0.25]), np.array([0.4, 0., 0.2])]
        R_all = [Ry(-np.pi), Ry(-np.pi), Ry(-np.pi), Ry(-np.pi), Ry(-np.pi), Ry(-np.pi)]
        elbow_all = [True, True, True, True, True, True]
    elif(no=="vuoto"):
        task=True
        c_all = [np.array([0.3, 0.2, 0.3]), np.array([-0.3, 0, 0.221]), np.array([-0.42, 0, 0.15]),np.array([0.3, 0.2, 0.3])]
        R_all = [Ry(np.pi),Ry(-np.pi), Ry(-np.pi), Ry(np.pi)]
        elbow_all = [True, True, True, True]
    elif(no == "prova"):
        task = True
        c_all = [np.array([0.4, 0., 0.2])]
        R_all = [Ry(np.pi)]
        elbow_all = [False]
    else:
        print("Give as input pirulo or paletta or prova")
    return task, c_all, R_all, elbow_all


# indenta tutto e togli la linea sotto
time_array_all = [60, 10, 10, 10, 10, 60]
task, c_all, R_all, elbow_all = premade_traj_time("vuoto")

q_tot = np.zeros(6)
for cont in range(len(c_all)):
    c = c_all[cont]
    R = R_all[cont]
    elbow = elbow_all[cont]
    q_tot = inv_kin_total_bis(c, R, task, elbow, q_tot[3:])
    print("Pose: ",q_tot)
    print("Cartesian position the real: ",dir_kin_total(q_tot,task)) 
    rot_d, coord_d = dir_kin_total_with_rot(q_tot,task)
    print("Cartesian position the desired: ",inv_kin_total_bis(coord_d, rot_d, task, elbow, q_tot[3:]))
    ########################################################################


    ## the initial trajectory is defined
    q_obj = [q_tot[0], q_tot[1], q_tot[2], q_tot[3], q_tot[4], q_tot[5]]
    time_array = np.array(list(range(time_array_all[cont])))
    T1 = rtb.jtraj(era.q, q_obj, time_array)
    T1.plot(True)
    #T1_u = rtb.mtraj(rtb.trapezoidal, era.q, [2.9370, 0.8761, 0.3903, -3.1416, 2.0566, -0.2046], 1000, V = 5/1000)

    #T1, k = scale_traj(T1, vel_lim)

    ## moving the simulated arm
    for j in range(0,math.ceil(time_array_all[cont])):
        
        pos = era.q
        #era.q = T.q[j]
        if j == 0:
            pos = pos + (T1.qd[j])/2
        else:
            pos = pos + (T1.qd[j]+T1.qd[j-1])/2

        #print(era.q)
        #print(T.q[j])

        era.q = T1.q[j]  ### prima pos
        env.step(1)

    print(era.fkine(era.q))

print(dir_kin_total(np.array([era.q[0], era.q[1], era.q[2], era.q[3], era.q[4], era.q[5]]), False))

time.sleep(5)
q_pres = [era.q[0], era.q[1], era.q[2], era.q[3], era.q[4], era.q[5]]
steps = 50
q = move_from_rtag('main', q_pres, steps) 
q = rotate(False, True, q_pres, 'r_cw', steps, 1.0) 
print(q)
for j in range(0,steps) :
    era.q = [q[j][0], q[j][1], q[j][2], q[j][3], q[j][4], q[j][5]]
    env.step(1/steps)
    time.sleep(0.1)



'''
## multiple viapoint trajectory definition

T2 = rtb.jtraj(era.q, [2.9279, 1.0360, 1.0164, -3.1416, 1.5904, -0.2137], 1000)
#T1.plot(True)

for j in range(0,1000) :
    
    pos = era.q
    #era.q = T.q[j]
    if j == 0:
        pos = pos + (T2.qd[j])*0.001/2
    else:
        pos = pos + (T2.qd[j]+T2.qd[j-1])*0.001/2

    print(era.q)
    #print(T.q[j])

    era.q = pos
    env.step(0.001)


T3 = rtb.jtraj(era.q, [2.9370, 0.8761, 0.3903, -3.1416, 2.0566, -0.2046], 1000)
#T1.plot(True)
sleep(3)

for j in range(0,1000) :
    
    pos = era.q
    #era.q = T.q[j]
    if j == 0:
        pos = pos + (T3.qd[j])*0.001/2
    else:
        pos = pos + (T3.qd[j]+T3.qd[j-1])*0.001/2

    print(era.q)
    #print(T.q[j])

    era.q = pos
    env.step(0.001)

T4 = rtb.jtraj(era.q, [0, 0.8808, 1.4897, -3.1416, 0.9619, -3.1416], 1000)
#T1.plot(True)

for j in range(0,1000) :
    
    pos = era.q
    #era.q = T.q[j]
    if j == 0:
        pos = pos + (T4.qd[j])*0.001/2
    else:
        pos = pos + (T4.qd[j]+T4.qd[j-1])*0.001/2

    print(era.q)
    #print(T.q[j])

    era.q = pos
    env.step(0.001)


T5 = rtb.jtraj(era.q, [0, 0.3756, 1.3132, -3.1416, 0.6332, -3.1416], 1000)
#T1.plot(True)

for j in range(0,1000) :
    
    pos = era.q
    #era.q = T.q[j]
    if j == 0:
        pos = pos + (T5.qd[j])*0.001/2
    else:
        pos = pos + (T5.qd[j]+T5.qd[j-1])*0.001/2

    print(era.q)
    #print(T.q[j])

    era.q = pos
    env.step(0.001)

t = 5000
if isinstance(t, int):
        tscal = 1.0
        ts = np.linspace(0, 1, t)  # normalized time from 0 -> 1
        tv = ts * t
q = np.vstack([T1.q, T2.q, T3.q, T4.q, T5.q])
qd = np.vstack([T1.qd, T2.qd, T3.qd, T4.qd, T5.qd])
qdd = np.vstack([T1.qdd, T2.qdd, T3.qdd, T4.qdd, T5.qdd])
print(q.shape)
T = Trajectory("miatraj", tv, q, qd, qdd)
T.plot(True)
'''
print(era.q)
print(era.fkine(era.q))
print(dir_kin_total_with_rot(q[9], False))

sleep(1000)

