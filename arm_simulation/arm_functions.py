import roboticstoolbox as rbt
from spatialmath import SE3
import time

def move(P1, robot, t, dir):
    P2 = P1
    inc = 0.1
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

    T1 = SE3(P1)
    T2 = SE3(P2)

    start_time = time.time()
    trajectory = robot.jtraj(T1=T1, T2=T2, t=t)
    print("--- %s seconds passed ---", (time.time()-start_time))
    robot.plot(trajectory.q, block=False, limits=[-1, 1, -1, 1, -1, 1])

    return P2
