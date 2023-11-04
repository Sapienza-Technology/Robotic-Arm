Installation
============

To use the 2023 Tech Arm code it is essential to install some packages.
Firts make sure to have installed python3.6 or higher.

.. code-block:: console

   $ python3 --version

Then check to be running Ubuntu 20.04.

.. code-block:: console

   $ lsb_release -a

To run the majority of the code and to comunicate with the arm motors, ROS Noetic is needed.
To install ROS Noetic follow the instructions on the official ROS website: http://wiki.ros.org/noetic/Installation/Ubuntu
Make sure to install the full desktop version of ROS Noetic:

.. code-block:: console

   $ sudo apt install ros-noetic-desktop-full

Another important package is MoveIt. To install it follow the instructions on the official MoveIt website: https://moveit.ros.org/install/
Make sure to install the version of MoveIt for ROS Noetic:

.. code-block:: console

   $ sudo apt-get install ros-noetic-moveit

An essential package to run the code is roboticstoolbox-python. To install it run the following command:

.. code-block:: console

   $ pip install roboticstoolbox-python

Morover, make sure to have some relevant python packages installed, if not use pip to install them:

.. code-block:: console

    $ pip install numpy
    $ pip install scipy
    $ pip install matplotlib
    $ pip install sympy
    $ pip install spatialmath-python
    $ pip install rospkg
    $ pip install catkin_pkg
    $ pip install opencv-python
    $ pip install swift-sim


To install the 2023 Tech Arm code, first clone the repository: 

.. code-block:: console

   $ git clone https://github.com/Sapienza-Technology/Robotic-Arm.git

Then, move to the rob_arm_ws folder and build the workspace:

.. code-block:: console

   $ cd Robotic-Arm/rob_arm_ws
   $ catkin build

Finally, source the workspace:

.. code-block:: console

   $ source devel/setup.bash

To test the installation, run the following command:

.. code-block:: console

   $ rosrun arm_main_pkg interactive_stupid_trajectory.launch

If everything is installed correctly, instructions on how to move the arm should appear in the terminal.