Docker Setup
============

This is a simple docker setup for running a local development environment
for the arm project.

The docker image is based on Ubuntu 20.04 and ROS Noetic.


On Ubuntu
---------

Install docker engine following the instructions here: https://docs.docker.com/engine/installation/linux/ubuntulinux/

After the installation is complete, download from the shared folder https://drive.google.com/drive/folders/1GU7_l9fJuTjuC4oRc309Y_ViFABsb78h?usp=sharing
the Docker file Dockerfile.sasa_dock, and run the following line on terminal:

.. code-block:: console

    $ docker build -t sasa_dock . -f ./Dockerfile.sasa_dock

This command will build the image of the SASA docker, naming it sasa_dock.
After completing the building operation, that will take a few minutes, the image will be in the
Docker Desktop application and it will be available to run in a container.

To create the container and open the docker bash, execute the following commands:

.. code-block:: console

    $ docker create --net=host --device=/dev/dri -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY --name arm_cont  sasa_dock
    $ docker exec -it arm_cont /bin/bash

Moreover, it can be useful to download also the Docker extention for VSCode, to navigate the container, but
the docker engine need the root permission to run, and VSCode does not have the root permission.

To allow VSCode to see the images and the containers, it is necessary to add the user to the docker group:

.. code-block:: console

    $ sudo usermod -aG docker $USER

After this, it is necessary to restart the computer to apply the changes.

If this does not solve the VSCode problem and the user is still not able to see the images and the containers,
it is necessary to change the docker deamon to run as non root user. \n
To change its permission follow the instructions here: https://docs.docker.com/engine/install/linux-postinstall/
and https://docs.docker.com/engine/security/rootless/ \n
After completing the procedure, the docker engine will run as non root user.

If executing docker engine, the following error comes out:

.. code-block:: console

    $ docker run hello-world

    out: error getting credentials - err: docker-credential-desktop resolves to executable in current directory (./docker-credential-desktop) ...

try to run the following command:

.. code-block:: console

    $ rm -rf ~/.docker 

it should solve it.


On Windows
----------

A similar procedure can be done on Windows, but with some final differences. \n
Install Docker Desktop from https://docs.docker.com/desktop/install/windows-install/
Make sure to follow the procedure described in the site, considering that it is more simple 
to activate WSL 2 for the Docker Engine to work, as described at the beginning of the installation.

After completing the installation procedure, download from the shared folder https://drive.google.com/drive/folders/1GU7_l9fJuTjuC4oRc309Y_ViFABsb78h?usp=sharing
the Docker file Dockerfile.sasa_dock, and run the following line on Windows Terminal, running it as 
Administator:

.. code-block:: console

    $ docker build -t sasa_dock . -f .\Dockerfile.sasa_dock

This command will build the image of the SASA docker, naming it sasa_dock.
After completing the building operation, that will take a few minutes, the image will be in the 
Docker Desktop application and it will be available to run in a container. \n
To create the container and open the docker bash, execute the following commands:

.. code-block:: console

    $ docker create --net=host --device=/dev/dri -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY --name arm_cont  sasa_dock
    $ docker exec -it arm_cont /bin/bash

Moreover, it can be useful to download also the Docker extention for VSCode, to navigate the container
in a fast way. 

It can happen that, executing the container, roscore does not run because the network is not configured 
as it should, resulting in ROS being uncapable of talking with the machine itself.
To solve this problem it is necessary to change the file root/.bashrc:

.. code-block:: console

    $ apt install nano
    $ hostname -I

    out: 192.168.61.5 ... (example)

choose the first address and change the file:

.. code-block:: console

    $ nano root/.bashrc

add at the end of the file the following lines:

.. code-block:: console

    export ROS_HOSTNAME=192.168.61.5
    export ROS_MASTER_URI=http://192.168.61.5:11311

save the file, close the container bash and reopen it. \n
Now, the container is ready to run roscore and the other ROS nodes.



