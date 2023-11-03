#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <stdio.h>
#include "std_msgs/Float32MultiArray.h"
//#include <cstdlib>

/*
    Useful links we took inspiration from:
    - https://github.com/SlateRobotics/tr1 (here they also use joint limits, could be useful in the future)
    - https://github.com/cyborg-x1/ros_control_test
*/

double cmd_temp[6] = {};
double pos_temp[6] = {};

class MyRobot : public hardware_interface::RobotHW {
    public:
        MyRobot() {
            const char* joints[] = {"giunto0", "giunto1", "giunto2", "giunto3", "giunto4", "giunto5"};
            //const char* joints[] = {"giunto0"};

            for (int i=0; i<6; i++) {
                // connect and register the joint state interface
                // Il controllore prende i dati da pos, vel e eff
                hardware_interface::JointStateHandle state_handle(joints[i], &pos[i], &vel[i], &eff[i]);
                jnt_state_interface.registerHandle(state_handle);

                // connect and register the joint position interface
                // Il controllore infila i dati in cmd. Quindi per comunicare con arduino devo prenderli da qui
                hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(joints[i]), &cmd[i]);
                jnt_pos_interface.registerHandle(pos_handle);

                // *** USARE UNO DI QUESTI SE VOGLIAMO CONTROLLARE IN VELOCITÀ O IN EFFORT ***
                // connect and register the joint velocity interface
                //hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(joints[i]), &cmd[i]);
                //jnt_vel_interface.registerHandle(vel_handle);

                // connect and register the joint effort interface
                //hardware_interface::JointHandle eff_handle(jnt_state_interface.getHandle(joints[i]), &cmd[i]);
                //jnt_eff_interface.registerHandle(eff_handle);
            }

            registerInterface(&jnt_state_interface);
            registerInterface(&jnt_pos_interface);
            //registerInterface(&jnt_vel_interface);
            //registerInterface(&jnt_eff_interface);
        }

        void read() {
            // Implementare con arduino
            // Prende i dati con arduino e li manda al controllore. Quindi devi metterli negli arrai pos, vel e eff
            //pos[0] = rand() % 10;
            //pos[0] = cmd[0] - 0.1*cmd[0];
            //printf("Pos: %f\n", pos[0]);
            int i = 0;
            for (i = 0; i < 6; i++) {
                pos[i] = cmd[i];
            }

        }

        void write() {
            // Implementare con arduino
            // Prende i dati dal controllore e li manda ad arduino. Quindi bisogna prenderli da cmd e inviarli con rosserial.
            //printf("cmd[0] = %f\n", cmd[0]);
        }

    //protected:
        // Devo usare solo una di queste interfacce. Quale usare dipende da cosa accettano i motori.
        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::PositionJointInterface jnt_pos_interface;
        //hardware_interface::VelocityJointInterface jnt_vel_interface;
        //hardware_interface::EffortJointInterface jnt_eff_interface;
        double cmd[2];
        double pos[2];
        double vel[2];
        double eff[2];
};
/*
int publisher(double pos) {
    ros::init("talker");
    
    ros::NodeHandle nh;
    double val[1] = {pos};
    std_msgs::Float32MultiArray msg;
    ros::Publisher pub("send_pos", &msg);

    nh.initNode();


} 
*/

void feedback_cb(const std_msgs::Float32MultiArray& msg) {
    printf("Feedback: [[%f], [%f], [%f], [%f], [%f], [%f]]", msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5]);
    int i = 0;
    for (i = 0; i < 6; i++) {
        pos_temp[i] = msg.data[i];
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "new_hardware_interface_node");
    //ros::NodeHandle nh("braccio_urdf");
    ros::NodeHandle nh("braccio_urdf");
    ros::CallbackQueue queue;
    nh.setCallbackQueue(&queue);

    //std::vector<float> command = {1.0};
    std::vector<float> command = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    std_msgs::Float32MultiArray cmd_msg;

    // Creiamo il publisher per inviare i dati ad arduino
    //ros:: Publisher chatter_pub = nh.advertise<std_msgs::Float32MultiArray>("arm_pos", 1000);
    ros:: Publisher chatter_pub = nh.advertise<std_msgs::Float32MultiArray>("/firmware_arm_pos", 1000);
    //ros::Subscriber sub = nh.subscribe("/arm_pos_feedback", 1000, feedback_cb);

    MyRobot robot;
    controller_manager::ControllerManager cm(&robot, nh);


    ros::AsyncSpinner spinner(4, &queue);
    spinner.start();

    ros::Time ts = ros::Time::now();

    ros::Rate rate(50);  //era 50


    while(ros::ok()) {
        ros::Duration d = ros::Time::now() - ts;
        ts = ros::Time::now();
        robot.read();
        cm.update(ts, d);
        //robot.write();
        //command[0] = robot.cmd[0];
        //command[3] = robot.cmd[3];
        //command[5] = robot.cmd[5];
        //command[3] = -800;
        //command[1] = 1600;
        for (int i = 0; i < 6; i++) {
            command[i] = robot.cmd[i];
        }
        cmd_msg.data = command;
        //cmd_msg.data_length = 1;
        chatter_pub.publish(cmd_msg);
        rate.sleep();
    } 

/*
    //command[3] = -800;
    //command[5] = 2000;
    command[1] = 1000;
    cmd_msg.data = command;
    //cmd_msg.data_length = 1;
    chatter_pub.publish(cmd_msg);
    rate.sleep();
*/    

    spinner.stop();

    return 0;
};

