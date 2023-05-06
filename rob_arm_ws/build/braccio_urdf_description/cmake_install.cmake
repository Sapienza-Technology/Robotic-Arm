# Install script for directory: /home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/src/braccio_urdf_description

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install" TYPE PROGRAM FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/braccio_urdf_description/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install" TYPE PROGRAM FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/braccio_urdf_description/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install/setup.bash;/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install" TYPE FILE FILES
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/braccio_urdf_description/catkin_generated/installspace/setup.bash"
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/braccio_urdf_description/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install/setup.sh;/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install" TYPE FILE FILES
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/braccio_urdf_description/catkin_generated/installspace/setup.sh"
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/braccio_urdf_description/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install/setup.zsh;/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install" TYPE FILE FILES
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/braccio_urdf_description/catkin_generated/installspace/setup.zsh"
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/braccio_urdf_description/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install" TYPE FILE FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/braccio_urdf_description/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/braccio_urdf_description/msg" TYPE FILE FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/src/braccio_urdf_description/msg/Float6Array.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/braccio_urdf_description/cmake" TYPE FILE FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/braccio_urdf_description/catkin_generated/installspace/braccio_urdf_description-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/devel/.private/braccio_urdf_description/include/braccio_urdf_description")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/devel/.private/braccio_urdf_description/share/roseus/ros/braccio_urdf_description")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/devel/.private/braccio_urdf_description/share/common-lisp/ros/braccio_urdf_description")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/devel/.private/braccio_urdf_description/share/gennodejs/ros/braccio_urdf_description")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/devel/.private/braccio_urdf_description/lib/python3/dist-packages/braccio_urdf_description")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/devel/.private/braccio_urdf_description/lib/python3/dist-packages/braccio_urdf_description")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/braccio_urdf_description/catkin_generated/installspace/braccio_urdf_description.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/braccio_urdf_description/cmake" TYPE FILE FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/braccio_urdf_description/catkin_generated/installspace/braccio_urdf_description-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/braccio_urdf_description/cmake" TYPE FILE FILES
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/braccio_urdf_description/catkin_generated/installspace/braccio_urdf_descriptionConfig.cmake"
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/braccio_urdf_description/catkin_generated/installspace/braccio_urdf_descriptionConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/braccio_urdf_description" TYPE FILE FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/src/braccio_urdf_description/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/braccio_urdf_description" TYPE PROGRAM FILES
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/src/braccio_urdf_description/scripts/robot_arm_script.py"
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/src/braccio_urdf_description/scripts/PID.py"
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/src/braccio_urdf_description/scripts/target_sender.py"
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/src/braccio_urdf_description/scripts/arm_functions.py"
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/src/braccio_urdf_description/scripts/getch.py"
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/src/braccio_urdf_description/scripts/ikine.py"
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/src/braccio_urdf_description/scripts/interactive_send_goal.py"
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/src/braccio_urdf_description/scripts/pid_test.py"
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/src/braccio_urdf_description/scripts/ramp.py"
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/src/braccio_urdf_description/scripts/robot_arm_script.py"
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/src/braccio_urdf_description/scripts/send_goal.py"
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/src/braccio_urdf_description/scripts/image_sender.py"
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/src/braccio_urdf_description/scripts/send_trajectory_final.py"
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/src/braccio_urdf_description/scripts/sasa_functions.py"
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/src/braccio_urdf_description/scripts/traj_genv6.py"
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/src/braccio_urdf_description/scripts/trajectory.py"
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/src/braccio_urdf_description/scripts/stupid_trajectory.py"
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/src/braccio_urdf_description/scripts/interactive_stupid_trajectory.py"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/braccio_urdf_description/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/braccio_urdf_description/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
