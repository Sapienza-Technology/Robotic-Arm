#!/usr/bin/env python3

from matplotlib.pyplot import step
from sympy import deg
import rospy
from std_msgs.msg import Float32MultiArray, Float32
import time
import math
from getch import getch
#from arm_functions import *

from traj_genv6 import *

def degToRad(angle):
    return angle/180*math.pi

def joint_by_joint_movement(character, joint_index, q_pres, step):
    i = joint_index - 1

    q_pres_int = np.array([q_pres[0], q_pres[1], q_pres[2], q_pres[3], q_pres[4], q_pres[5]])
    if character == 'w':
        q_pres_int[i] = q_pres_int[i] + step
    elif character == 's':
        q_pres_int[i] = q_pres_int[i] - step

    arm_goal_arr = [q_pres_int]

    return arm_goal_arr

def pinza_movement(character, pinza_pres, step):

    if character == 'w':
        pinza_pres = pinza_pres + step
    elif character == 's':
        pinza_pres = pinza_pres - step

    pinza_goal = pinza_pres

    return pinza_goal


def traj_movement(character, q_pres, offsets, steps, delta, q_gripper=0.0, in_final_frame = False):
    if character == 'w': 
        #p1 = move_gazebo(p1, 'up')
        arm_goal_arr = go_straight(True, True, q_pres, "u", steps, delta, q_gripper, in_final_frame)
    elif character == 's': 
        arm_goal_arr = go_straight(True, True, q_pres, "d", steps, delta, q_gripper, in_final_frame)
    elif character == 'd': 
        arm_goal_arr = go_straight(True, True, q_pres, "r", steps, delta, q_gripper, in_final_frame)
    elif character == 'a': 
        arm_goal_arr = go_straight(True, True, q_pres, "l", steps, delta, q_gripper, in_final_frame)
    elif character == 'q': 
        arm_goal_arr = go_straight(True, True, q_pres, "f", steps, delta, q_gripper, in_final_frame)
    elif character == 'e': 
        arm_goal_arr = go_straight(True, True, q_pres, "b", steps, delta, q_gripper, in_final_frame)
    # for rotations
    elif character == '6': 
        arm_goal_arr = rotate(True, True, q_pres, "r_cw", steps, delta*10, q_gripper, False)
    elif character == '4': 
        arm_goal_arr = rotate(True, True, q_pres, "r_ccw", steps, delta*10, q_gripper, False)
    elif character == '2': 
        arm_goal_arr = rotate(True, True, q_pres, "p_cw", steps, delta*10, q_gripper, False)
    elif character == '8': 
        arm_goal_arr = rotate(True, True, q_pres, "p_ccw", steps, delta*10, q_gripper, False)
    elif character == '3':
        arm_goal_arr = rotate(True, True, q_pres, "y_cw", steps, delta*10, q_gripper, False)
    elif character == '1': 
        arm_goal_arr = rotate(True, True, q_pres, "y_ccw", steps, delta*10, q_gripper, False)
    elif character == 'z':
        arm_goal_arr = [np.zeros(6) + offsets]
    else:
        return -1

    return arm_goal_arr


def main():
    #### we define ros node and publishers
    rospy.init_node("stupid_trajectory")

    pub_command = rospy.Publisher("/firmware_arm_pos", Float32MultiArray, queue_size=100)
    pub_command_vel = rospy.Publisher("/firmware_arm_vel", Float32MultiArray, queue_size=100)
    pub_ee = rospy.Publisher("/firmware_arm_pinza", Float32, queue_size=100)

    #### we define the trajectory hyperparameters
    total_time = 2
    steps = 150
    max_time = total_time/steps

    offsets = np.array([0, degToRad(120), degToRad(-120), 0, 0, 0])
    q_pres = np.array([0, degToRad(120), degToRad(-120), 0, 0, 0])
    pinza_angle = 0
    in_final_frame = False


    def send_command(arm_goal_arr, joint_val_pres, max_time):
        last_command = joint_val_pres-offsets
        for i in range(len(arm_goal_arr)):
            command = Float32MultiArray()
            command_vel = Float32MultiArray()

            print(f"posizione richiesta prestampa: {arm_goal_arr[i]}")
            print(f"posizione richiesta prestampa: {arm_goal_arr[i][:3]-np.array([0.001,0.001,0.001])}")
            res1, bool_vec1 = range_check(arm_goal_arr[i][:3]-np.array([0.001,0.001,0.001]))
            res2, bool_vec2 = range_check(arm_goal_arr[i][:3]+np.array([0.001,0.001,0.001]))
            bool_vec = bool_vec1+bool_vec2
            print(res1, res2)
            print("\n")

            if not res1 or not res2 :
                print("Range check failed: joint " + str(np.where(bool_vec == 1)[0]+1) + " out of range")
                print("Per muovere comunque gli altri giunti premi g \n altrimenti premi qualunque altro pulsante")
                print("posizione richiesta: " + str(arm_goal_arr[i]))
                character = getch()
                print(f"last command: {last_command}")
                if character == 'g':
                    command.data = arm_goal_arr[i] - offsets
                    last_command = command.data
                else:
                    command.data = last_command
            else:
                command.data = arm_goal_arr[i] - offsets
                last_command = command.data
            
            print(f"offsets: {offsets}")
            #command.data = arm_goal_arr[i] - offsets
            delta_joint = command.data - joint_val_pres
            joint_val_pres = command.data
            command_vel.data = delta_joint/max_time
            pub_command_vel.publish(command_vel)
            pub_command.publish(command)
            print(f"command sent: {command.data}")
            print("command vel sent: " + str(command_vel.data))
            print(np.degrees(arm_goal_arr[i]))
            time.sleep(max_time)

        return last_command + offsets

    is_traj = True

    traj_mov_list = ['w', 's', 'd', 'a', 'q', 'e', '6', '4', '2', '8', '3', '1', 'z']

    character=''
    
    while(True):
        print("Per muovere un giunto alla volta premere 'm' \nPer muovere il braccio in una traiettoria premere 't' \n Per muovere il gripper premere 'p' \nPer uscire premere 'c'")

        character = getch()
        print(character)   

        if character == 'c':
            return  
        
        elif character == 'm':
            # Movimento giunto per giunto

            while (True):
                # Scelta del giunto da muovere
                print("Inserire un numero da 1 a 6")
                character = getch()
                print(character)

                if (character == 'c'):
                    break
                elif (character < '1' or character > '6'):
                    print("Inserire un numero da 1 a 6")
                    continue

                # Giunto scelto correttamente
                joint_index = int(character)
                print("Inserire 'w' per muovere il giunto in avanti e 's' per muoverlo indietro. Inserire 'c' per scegliere un altro giunto")

                while (True):
                    character = getch()
                    print(character)
                    # Scegliere un altro giunto
                    if (character == 'c'):
                        break
                    elif (character != 'w' and character != 's'):
                        print("Inserire 'w' per muovere il giunto in avanti e 's' per muoverlo indietro")
                    else:
                        arm_goal_arr = joint_by_joint_movement(character, joint_index, q_pres, degToRad(1))
                        print(arm_goal_arr)
                        #q_pres_jj = arm_goal_arr[-1]
                        q_pres = send_command(arm_goal_arr, q_pres, max_time*0.8)
                        
        elif character == 't':
            # Movimento in traiettoria
            print("Inserire:\t\t'w' per muoversi in alto \n\t's' per muoversi in basso \n\t'a' per muoversi a sinistra \n\t'd' per muoversi a destra \n\t'q' per muovers in avanti \n\t'e' per muoversi indietro \n\t'z' per tornare alla posizione iniziale \n\t'c' per tornare alla selezione modalità\n ----------- \n\t 'f' per aumentare la velocità \n\t 'g' per diminuire la velocità \n\t 'h' per diminuire ancora la velocità \n\t 'm' per muoversi nel frame finale \n\t 'n' per muoversi nel frame di base")  
            delta = 0.01
            while (True):
                character = getch()
                print(character)
                if (character == 'c'):
                    break
                elif (character == 'f'):
                    delta = 0.05
                elif (character == 'g'):
                    delta = 0.01
                elif (character == 'h'):
                    delta = 0.0025
                elif (character == 'm'):
                    in_final_frame = True
                elif (character == 'n'):
                    in_final_frame = False
                elif (character in traj_mov_list):
                    arm_goal_arr = traj_movement(character, q_pres, offsets, steps, delta, pinza_angle, in_final_frame)   
                    #q_pres = arm_goal_arr[-1]
                    q_pres = send_command(arm_goal_arr, q_pres, max_time*0.8)
                else:
                    print("Inserire:\n\t'w' per muoversi in alto \n\t's' per muoversi in basso \n\t'a' per muoversi a sinistra \n\t'd' per muoversi a destra \n\t'q' per muovers in avanti \n\t'e' per muoversi indietro \n\t'z' per tornare alla posizione iniziale \n\t'c' per tornare alla selezione modalità\n ----------- \n\t 'f' per aumentare la velocità \n\t 'g' per diminuire la velocità \n\t 'h' per diminuire ancora la velocità \n\t 'm' per muoversi nel frame finale \n\t 'n' per muoversi nel frame di base")  
        
        elif character == 'p':
            # Movimento pinza
            print("Inserire:\n\t'w' per aprire pinza \n\t's' per chiuderla")  
            
            while (True):
                character = getch()
                print(character)
                if (character == 'c'):
                    break
                elif (character):
                    pinza_angle = pinza_movement(character, pinza_angle, degToRad(20))
                    #q_pres = arm_goal_arr[-1]
                    command_pinza = Float32()
                    command_pinza.data = pinza_angle
                    pub_ee.publish(command_pinza)
                    print(f"command sent: {command_pinza.data}")
                    #time.sleep(1)
                else:
                    print("Inserire:\n\t'w' per muoversi in alto \n\t's' per muoversi in basso ")

        else:
            print("Inserire:\n\t'm' per muovere un giunto alla volta \n\t't' per muovere il braccio in una traiettoria \n\t'c' per uscire")
        """
        elif character == "o":
            command = Float32()
            command.data = 0
            pub_ee.publish(command)
            time.sleep(1)
            continue
        elif character == "p":
            command = Float32()
            command.data = 1
            pub_ee.publish(command)
            time.sleep(1)
            continue
        elif character == "k":
            arm_goal_arr = q_pres.copy()
            arm_goal_arr[5] += degToRad(3)
        elif character == "l":
            arm_goal_arr = q_pres.copy()
            arm_goal_arr[5] -= degToRad(3)
        """

        """
        q_pres = arm_goal_arr[-1]

        for i in range(len(arm_goal_arr)):
            command = Float32MultiArray()
            command_vel = Float32MultiArray()
            command.data = arm_goal_arr[i] - offsets
            delta_joint = command.data - joint_val_pres
            joint_val_pres = command.data
            command_vel.data = delta_joint/max_time
            pub_command.publish(command)
            print(f"command sent: {command.data}")
            pub_command_vel.publish(command_vel)
            print(np.degrees(arm_goal_arr[i]))
            time.sleep(max_time)
        """

        

main()       