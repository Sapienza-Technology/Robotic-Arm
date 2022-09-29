# Robotic-Arm

This repo contains the code for movement and control of our robotic arm.

The folder called rob_arm_ws contains the catkin workspace we are working in. src/braccio_urdf_description is the folder containing the catkin package called *braccio_urdf_description* (we should change this name to something meaningful).

This is the package containing all the code controlling the robotic arm.

The files written in python are contained in src/braccio_urdf_description. This is where the majority of the code executed in the ground station (our pc) is stored. The most important files at the moment are:
    - *interactive_stupid_trajectory.py*: this is the code used during the competition. It is used to manually control the end-effector using WASD and a bunch of other keys. The term stupid is used to indicate the fact that we are not using any feedback controller, which was our initial plan. In this code we are just sending to the Arduino a sequence of positions (trajectory) that the end-effector should reach and trust that they will be reached, without making any checks. We can do this because the motors are controlled through drivers equipped with an internal feedback loop that should make sure that the joints reach the positions they are given.
    - *stupid_trajectory.py*: this is the code we use to make the robot arm follow a predefined trajectory. It follows the same behavior of *interactive_stupid_trajectory.py*, meaning that we don't perform any active control on the executed trajectories through software.
    - *traj_genv6.py*: this file contains the code that generates all the trajectories used in *stupid_trajectory.py*.
    - *sasa_functions.py*: this file mainly contains functions used to compute transformations used to move the robotic arm. Some of these functions are used in *interactive_stupid_trajectory.py*.
    - *interactive_send_goal.py* and *send_goal.py*: these files contain the code that has the same behavior as *interactive_stupid_trajectory.py* and *stupid_trajectory.py* but using an actual PID controller. In this case the trajectory is inserted into a Goal object which is then sent to the controller which in turn manages how to send them to the Arduino based on the feedback coming from the joints. The code for this controller is written in C++ and is stored in *src/braccio_urdf_description/hardware_interface/new_hardware_interface_node.cpp*. In the end we didn't use this because the final arm doesn't have a way of returning the actual position of the joints.
    The executable file corresponding to *new_hardware_interface_node.cpp* is *devel/lib/braccio_urdf_description/braccio_urdf_description.cpp*. To execute it, it's necessary to also activate the controllers of the joints by launching *controller.launch*. 