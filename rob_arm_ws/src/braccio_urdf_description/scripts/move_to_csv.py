#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32

from move_to_calibration import send3DGoal, degToRad

diz = {"arPos": [0, 0, 0]}

#targetPos = [0, 0, 0]

def callback(msg, targetPos, sent, diz):
    if sent:
        diz["arPos"] = msg.data
        #print(f"detected pos: {arPos} - given pos: {targetPos}")

def main():
    rospy.init_node("move_to_csv")

    targetPos = [0, 0, 0]
    sent = False

    #rospy.Subscriber("/marker_poses", Float32MultiArray, lambda x: callback(x, targetPos, sent))
    rospy.Subscriber("/arm_pos_feedback", Float32MultiArray, lambda x: callback(x, targetPos, sent, diz))
    print("subscribed to marker_poses")

    pub_command = rospy.Publisher("/firmware_arm_pos", Float32MultiArray, queue_size=100)
    pub_ee = rospy.Publisher("/firmware_EEM_pos", Float32, queue_size=100)

    offsets = np.array([0, degToRad(90), degToRad(-90), 0, -degToRad(115), 0])

    q_pres = offsets

    steps = 2

    #while True:
    
    with open("calibrationMeasures.csv", "w+") as f:
        f.write(f"Joint positions, Position from Zed\n")



    with open("calibrationMeasures.csv", "a") as f:
        while True:
            f.write(f"{targetPos}, {diz['arPos']}\n")

            print("Write three numbers separated by spaces representing the x y z coordinates of the target position")
            points = input()
            if points == 'c':
                break

            sent = True

            targetPos = send3DGoal(points, pub_command)
            









    rospy.spin()



        

if __name__ == "__main__":
    main()

