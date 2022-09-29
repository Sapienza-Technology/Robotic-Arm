#!/usr/bin/env python3

import roboticstoolbox as rbt
from spatialmath import SE3
from spatialmath.base import transl, trotx, troty, trotz

import time

def move(P1, robot, t, dir):
    print('move iniziata')
    P2 = P1
    inc = 0.1
    inc_deg = [0.5,0.5,0.5]
    if (dir == 'up'):
        P2[2][-1] += inc
    elif (dir == 'down'):
        P2[2][-1] -= inc
    elif (dir == 'right'):
        P2[1][-1] -= inc
    elif (dir == 'left'):
        P2[1][-1] += inc
    elif (dir == 'forward'):
        P2[0][-1] += inc
    elif (dir == 'backwards'):
        P2[0][-1] -= inc

    elif (dir == 'pitch-up'):
        P2 = P2 @ troty(inc_deg[1])
    elif (dir == 'pitch-down'):
        P2 = P2 @ troty(-inc_deg[1])
    elif (dir == 'yaw-right'):
        P2 = P2 @ trotz(-inc_deg[2])
    elif (dir == 'yaw-left'):
        P2 = P2 @ trotz(inc_deg[2])
    elif (dir == 'roll-ccw'):
        P2 = P2 @ trotx(inc_deg[0])
    elif (dir == 'roll-cw'):
        P2 = P2 @ trotx(-inc_deg[0])

    T1 = SE3(P1)
    T2 = SE3(P2)

    start_time = time.time()
    trajectory = robot.jtraj(T1=T1, T2=T2, t=t)
    print("--- %s seconds passed ---", (time.time()-start_time))
    print(trajectory.q)
    robot.plot(trajectory.q, block=False, limits=[-1, 1, -1, 1, -1, 1])

    return P2
