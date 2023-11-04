Usage
=====

Direct Connection via Serial-USB
--------------------------------

To connect to the arm and move it, you need to establish a serial connection.

First initialize a ROS master:

.. code-block:: console

   $ roscore

Then you need to start the arm controller on the board.
Connect to the board via USB. Then you need to find out which port it is connected to. On Linux, you can do this by running 

.. code-block:: console

   $ dmesg | grep tty

or 

.. code-block:: console

   $ ls /dev/tty*

The target is eather `/dev/ttyUSB0` or `/dev/ttyACM0`.

Then you can connect to the board using rosserial:

.. code-block:: console

   $ rosrun rosserial_python serial_node.py /dev/ttyUSB0`

or

.. code-block:: console

   $ rosrun rosserial_python serial_node.py /dev/ttyACM0`

If rosserial is not installed, you can install it using:

.. code-block:: console

   $ sudo apt-get install ros-noetic-rosserial-python

After that, you can try to move the arm through interactive_stupid_trajectory or stupid_trajectory routines.

.. code-block:: console

   $ rosrun arm_main_pkg stupid_trajectory.py


Connection via WiFi and SSH
---------------------------

To connect to the arm and move it, you need to connect first to the jetson via ssh.

.. code-block:: console

   $ ssh jetson@[IP-jetson]

