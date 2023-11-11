Installation
============

The installation can be done on a virtual machine or on a real machine. 
The installation on a virtual machine is recommended on VMware, because the Oracle VM gives problems with the terminal in Ubuntu.m\\

To use the 2023 Tech Arm code it is essential to install some packages.
Firts make sure to have installed python3.6 or higher.

.. code-block:: console

   $ python3 --version

Then check to be running Ubuntu 20.04.

.. code-block:: console

   $ lsb_release -a

To run the majority of the code and to comunicate with the arm motors, ROS Noetic is needed.
To install ROS Noetic follow the instructions on the official ROS website: http://wiki.ros.org/noetic/Installation/Ubuntu
Make sure to install the full desktop version of ROS Noetic. \n

Another important package is MoveIt. To install it follow the instructions on the official MoveIt website: https://moveit.ros.org/install/
Make sure to install the version of MoveIt for ROS Noetic:

.. code-block:: console

   $ sudo apt-get install ros-noetic-moveit

To install the python packages, first installo the pip package manager:

.. code-block:: console

   $ sudo apt install python3-pip

Make sure to install some relevant python packages, use pip to install them:

.. code-block:: console

   $ pip3 install -U roboticstoolbox-python numpy scipy sympy qpsolvers matplotlib spatialmath-python rospkg catkin_pkg catkin-tools opencv-python swift-sim


Then you need to clone a GitHub repository on your PC, containing 2023 Tech Arm code.

First install git:

.. code-block:: console

   $ sudo apt install git

Then clone the repository: 

.. code-block:: console

   $ git clone https://github.com/Sapienza-Technology/Robotic-Arm.git

To build the project you need to install catkin build:

.. code-block:: console

   $ sudo apt-get install python3-catkin-tools

Then, move to the rob_arm_ws folder and build the workspace:

.. code-block:: console

   $ cd Robotic-Arm/rob_arm_ws
   $ catkin clean -y
   $ catkin build

If the catkin does not build correctly giving the error:

.. code-block:: console

   $ The catkin Cmake module was not found ....

probably you have not sourced the ROS noetic setup.bash file. To do so, run the following command:

.. code-block:: console

   $ source /opt/ros/noetic/setup.bash

or add the same line at the end of the file ~/.bashrc, opening it with nano:

.. code-block:: console

   $ nano ~/.bashrc


To test the installation, first open a terminal and run roscore

.. code-block:: console

   $ roscore

Then run the following commands in a second window:

.. code-block:: console

   $ source devel/setup.bash
   $ rosrun arm_main_pkg interactive_stupid_trajectory.py

If everything is installed correctly, instructions on how to move the arm should appear in the terminal.