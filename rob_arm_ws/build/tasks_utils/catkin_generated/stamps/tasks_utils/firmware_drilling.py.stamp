#!/usr/bin/env python3
"""
ROS node that waits for input of the user and send the command to the firmware


send a message using a Float32MultiArray with 3 numbers:
Stepper position, stepper speed, drilling speed [0 to 1]

"""

import rospy
from std_msgs.msg import Float32MultiArray

import sys, select, termios, tty


def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)   
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    rospy.init_node('firmware_drilling', anonymous=True)
    pub = rospy.Publisher('drilling_commands', Float32MultiArray, queue_size=10)
    msg = """
    Drilling control!
    ---------------------------

        I and K arrow:  move stepper up and down
        if D (drill) is pressed: increase drilling speed
        if S (Stepper) is pressed:increase stepper speed
        + or - : switch speed increase with speed decrease
        I to invert drilling rotation
        SPACE to start and stop the drill
    q to quit
    """
    settings=termios.tcgetattr(sys.stdin)
    print(msg)
    drill_msg=[0,0,0]
    stepper_speed = 10 #Step per second
    drill_speed = 0.1
    rate = rospy.Rate(30) # 10hz
    increasing = True
    stop=True
    sign_drill = 1 #move clockwise or counterclockwise
    while not rospy.is_shutdown():
        pressed=False
        key = getKey(settings)
        if key == 'i':
            drill_msg[0] += stepper_speed
            pressed=True
        elif key == 'k':
            drill_msg[0] -= stepper_speed
            pressed=True
        elif key == 'd':
            #get char to see if + or -
            if increasing:
                drill_speed *= 1.1
            else: 
                drill_speed /= 1.1
            drill_speed=sign_drill*drill_speed
            print("Drill speed: ", drill_speed)
            pressed=True
        elif key == 's':
            if increasing:
                stepper_speed *= 1.1
            else:
                stepper_speed /= 1.1
            print("Stepper speed: ", stepper_speed)
            pressed=True
        elif key == ' ':
            if stop:
                print("START!")
                stop=False
            else:
                print("STOP!")
                stop=True
            pressed=True
        elif key == '+':
            increasing = True
            print("increasing mode")
        elif key == '-':
            increasing = False
            print("decreasing mode")
        elif key == 'i':
            print("inverting drilling rotation")
            sign_drill *= -1
            #do not stop and invert all together, send first stop
            print("stopping drill")
            ros_msg = Float32MultiArray()
            ros_msg.data = [drill_msg[0], drill_msg[1], 0]
            pub.publish(ros_msg)
            time.sleep(1)
            print("inverting drill rotation")
            
        elif key == 'q':
            print("Exit..")
            #stop
            ros_msg = Float32MultiArray()
            ros_msg.data = [drill_msg[0], 0, 0]
            pub.publish(ros_msg)

            break
        else:
            if (key == '\x03'):
                break
        if stop:
            ros_msg = Float32MultiArray()
            ros_msg.data = [drill_msg[0], drill_msg[1], 0]
            pub.publish(ros_msg)
        else:
            if pressed:
                print("Stepper position: ", drill_msg[0], "Stepper speed: ", stepper_speed, "Drill speed: ", drill_speed)
                ros_msg = Float32MultiArray()

                drill_msg[1] = stepper_speed
                drill_msg[2] = drill_speed
                ros_msg.data = drill_msg
                pub.publish(ros_msg)
    

if __name__ == '__main__':
    main()