#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float64
from braccio_urdf_description.msg import Float6Array
import math
from pid_test import controller_pid

#****** Provare ad usare il package simple-pid *************************

threshold = 0.001
feedback = []
flag = 0    # flag = 1 se sono arrivate le posizioni effettive che il braccio ha assunto dal comando precedentemente inviato
            # cioè quando viene eseguita callback_true

def callback_true(data):
    rospy.loginfo(data)
    feedback = data
    flag = 1

# per ora il pid prende una posizione, noi vogliamo fargli prendere una velocità. Per farlo basterebbe integrare la velocità
# e ottenere la posizione
def callback_target(data, pub_command, rate_command, pub_ack, rate_ack):
    rospy.loginfo(data)
    pid = controller_pid(data)
    errors = [2*math.pi for x in range(6)]
    q_command = []
    while math.sqrt(sum(x**2 for x in errors)) < threshold:
        if flag == 1:
            for i in range(feedback):
                errors[i] = data[i] - feedback[i]
                q_command[i] = pid(feedback[i])
            flag = 0
        else:
            continue
        pub_command.publish(q_command)
        rate_command.sleep()
    # quando usciamo dal while abbiamo raggiunto la posizione desiderata

def callback_target_test(data):
    # adesso questo riceve un array, non un float, quindi bisogna cambiare questa condizione
    if data != 10.0:
        rospy.loginfo(data)
    else:
        print("ricevuto 10")
        while True:
            pub_ack.publish("ciccio")
            rate_ack.sleep()


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)

    pub_command = rospy.Publisher('/braccio_urdf/q_command', Float64MultiArray, queue_size=10) # posizione mandata al braccio
    rate_command = rospy.Rate(30)
    pub_ack = rospy.Publisher('/braccio_urdf/q_ack', Float64, queue_size=10)
    rate_ack = rospy.Rate(30)

    rospy.Subscriber('/braccio_urdf/joint_states', JointState, callback_true)
    #rospy.Subscriber('/braccio_urdf/q_target', Float64MultiArray, lambda x: callback_target(x, pub_command, rate_command, pub_ack, rate_ack))
    # prima vediamo se arrivano i messaggi
    rospy.Subscriber('/braccio_urdf/q_target', Float6Array, callback_target_test)

    rospy.spin()