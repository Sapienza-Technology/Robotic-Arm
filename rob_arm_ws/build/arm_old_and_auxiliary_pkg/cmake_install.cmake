# Install script for directory: /home/techsapienza/Robotic-Arm/rob_arm_ws/src/arm_old_and_auxiliary_pkg

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/techsapienza/Robotic-Arm/rob_arm_ws/install")
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
   "/home/techsapienza/Robotic-Arm/rob_arm_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/techsapienza/Robotic-Arm/rob_arm_ws/install" TYPE PROGRAM FILES "/home/techsapienza/Robotic-Arm/rob_arm_ws/build/arm_old_and_auxiliary_pkg/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/techsapienza/Robotic-Arm/rob_arm_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/techsapienza/Robotic-Arm/rob_arm_ws/install" TYPE PROGRAM FILES "/home/techsapienza/Robotic-Arm/rob_arm_ws/build/arm_old_and_auxiliary_pkg/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/techsapienza/Robotic-Arm/rob_arm_ws/install/setup.bash;/home/techsapienza/Robotic-Arm/rob_arm_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/techsapienza/Robotic-Arm/rob_arm_ws/install" TYPE FILE FILES
    "/home/techsapienza/Robotic-Arm/rob_arm_ws/build/arm_old_and_auxiliary_pkg/catkin_generated/installspace/setup.bash"
    "/home/techsapienza/Robotic-Arm/rob_arm_ws/build/arm_old_and_auxiliary_pkg/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/techsapienza/Robotic-Arm/rob_arm_ws/install/setup.sh;/home/techsapienza/Robotic-Arm/rob_arm_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/techsapienza/Robotic-Arm/rob_arm_ws/install" TYPE FILE FILES
    "/home/techsapienza/Robotic-Arm/rob_arm_ws/build/arm_old_and_auxiliary_pkg/catkin_generated/installspace/setup.sh"
    "/home/techsapienza/Robotic-Arm/rob_arm_ws/build/arm_old_and_auxiliary_pkg/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/techsapienza/Robotic-Arm/rob_arm_ws/install/setup.zsh;/home/techsapienza/Robotic-Arm/rob_arm_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/techsapienza/Robotic-Arm/rob_arm_ws/install" TYPE FILE FILES
    "/home/techsapienza/Robotic-Arm/rob_arm_ws/build/arm_old_and_auxiliary_pkg/catkin_generated/installspace/setup.zsh"
    "/home/techsapienza/Robotic-Arm/rob_arm_ws/build/arm_old_and_auxiliary_pkg/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/techsapienza/Robotic-Arm/rob_arm_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/techsapienza/Robotic-Arm/rob_arm_ws/install" TYPE FILE FILES "/home/techsapienza/Robotic-Arm/rob_arm_ws/build/arm_old_and_auxiliary_pkg/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_old_and_auxiliary_pkg/msg" TYPE FILE FILES "/home/techsapienza/Robotic-Arm/rob_arm_ws/src/arm_old_and_auxiliary_pkg/msg/Float6Array.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_old_and_auxiliary_pkg/cmake" TYPE FILE FILES "/home/techsapienza/Robotic-Arm/rob_arm_ws/build/arm_old_and_auxiliary_pkg/catkin_generated/installspace/arm_old_and_auxiliary_pkg-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/techsapienza/Robotic-Arm/rob_arm_ws/devel/.private/arm_old_and_auxiliary_pkg/include/arm_old_and_auxiliary_pkg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/techsapienza/Robotic-Arm/rob_arm_ws/devel/.private/arm_old_and_auxiliary_pkg/share/roseus/ros/arm_old_and_auxiliary_pkg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/techsapienza/Robotic-Arm/rob_arm_ws/devel/.private/arm_old_and_auxiliary_pkg/share/common-lisp/ros/arm_old_and_auxiliary_pkg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/techsapienza/Robotic-Arm/rob_arm_ws/devel/.private/arm_old_and_auxiliary_pkg/share/gennodejs/ros/arm_old_and_auxiliary_pkg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/techsapienza/Robotic-Arm/rob_arm_ws/devel/.private/arm_old_and_auxiliary_pkg/lib/python3/dist-packages/arm_old_and_auxiliary_pkg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/techsapienza/Robotic-Arm/rob_arm_ws/devel/.private/arm_old_and_auxiliary_pkg/lib/python3/dist-packages/arm_old_and_auxiliary_pkg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/techsapienza/Robotic-Arm/rob_arm_ws/build/arm_old_and_auxiliary_pkg/catkin_generated/installspace/arm_old_and_auxiliary_pkg.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_old_and_auxiliary_pkg/cmake" TYPE FILE FILES "/home/techsapienza/Robotic-Arm/rob_arm_ws/build/arm_old_and_auxiliary_pkg/catkin_generated/installspace/arm_old_and_auxiliary_pkg-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_old_and_auxiliary_pkg/cmake" TYPE FILE FILES
    "/home/techsapienza/Robotic-Arm/rob_arm_ws/build/arm_old_and_auxiliary_pkg/catkin_generated/installspace/arm_old_and_auxiliary_pkgConfig.cmake"
    "/home/techsapienza/Robotic-Arm/rob_arm_ws/build/arm_old_and_auxiliary_pkg/catkin_generated/installspace/arm_old_and_auxiliary_pkgConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_old_and_auxiliary_pkg" TYPE FILE FILES "/home/techsapienza/Robotic-Arm/rob_arm_ws/src/arm_old_and_auxiliary_pkg/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/arm_old_and_auxiliary_pkg" TYPE PROGRAM FILES
    "/home/techsapienza/Robotic-Arm/rob_arm_ws/src/arm_old_and_auxiliary_pkg/scripts/robot_arm_script.py"
    "/home/techsapienza/Robotic-Arm/rob_arm_ws/src/arm_old_and_auxiliary_pkg/scripts/PID.py"
    "/home/techsapienza/Robotic-Arm/rob_arm_ws/src/arm_old_and_auxiliary_pkg/scripts/target_sender.py"
    "/home/techsapienza/Robotic-Arm/rob_arm_ws/src/arm_old_and_auxiliary_pkg/scripts/arm_functions.py"
    "/home/techsapienza/Robotic-Arm/rob_arm_ws/src/arm_old_and_auxiliary_pkg/scripts/getch.py"
    "/home/techsapienza/Robotic-Arm/rob_arm_ws/src/arm_old_and_auxiliary_pkg/scripts/ikine.py"
    "/home/techsapienza/Robotic-Arm/rob_arm_ws/src/arm_old_and_auxiliary_pkg/scripts/interactive_send_goal.py"
    "/home/techsapienza/Robotic-Arm/rob_arm_ws/src/arm_old_and_auxiliary_pkg/scripts/pid_test.py"
    "/home/techsapienza/Robotic-Arm/rob_arm_ws/src/arm_old_and_auxiliary_pkg/scripts/ramp.py"
    "/home/techsapienza/Robotic-Arm/rob_arm_ws/src/arm_old_and_auxiliary_pkg/scripts/robot_arm_script.py"
    "/home/techsapienza/Robotic-Arm/rob_arm_ws/src/arm_old_and_auxiliary_pkg/scripts/send_goal.py"
    "/home/techsapienza/Robotic-Arm/rob_arm_ws/src/arm_old_and_auxiliary_pkg/scripts/image_sender.py"
    "/home/techsapienza/Robotic-Arm/rob_arm_ws/src/arm_old_and_auxiliary_pkg/scripts/send_trajectory_final.py"
    "/home/techsapienza/Robotic-Arm/rob_arm_ws/src/arm_old_and_auxiliary_pkg/scripts/sasa_functions.py"
    "/home/techsapienza/Robotic-Arm/rob_arm_ws/src/arm_old_and_auxiliary_pkg/scripts/traj_genv6.py"
    "/home/techsapienza/Robotic-Arm/rob_arm_ws/src/arm_old_and_auxiliary_pkg/scripts/trajectory.py"
    "/home/techsapienza/Robotic-Arm/rob_arm_ws/src/arm_old_and_auxiliary_pkg/scripts/stupid_trajectory.py"
    "/home/techsapienza/Robotic-Arm/rob_arm_ws/src/arm_old_and_auxiliary_pkg/scripts/interactive_stupid_trajectory.py"
    "/home/techsapienza/Robotic-Arm/rob_arm_ws/src/arm_old_and_auxiliary_pkg/scripts/prova_movit_dock.py"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/techsapienza/Robotic-Arm/rob_arm_ws/build/arm_old_and_auxiliary_pkg/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/techsapienza/Robotic-Arm/rob_arm_ws/build/arm_old_and_auxiliary_pkg/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
