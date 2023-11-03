# Install script for directory: /home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/src/tasks_utils

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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install" TYPE PROGRAM FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tasks_utils/catkin_generated/installspace/_setup_util.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install" TYPE PROGRAM FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tasks_utils/catkin_generated/installspace/env.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install/setup.bash;/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install" TYPE FILE FILES
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tasks_utils/catkin_generated/installspace/setup.bash"
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tasks_utils/catkin_generated/installspace/local_setup.bash"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install/setup.sh;/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install" TYPE FILE FILES
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tasks_utils/catkin_generated/installspace/setup.sh"
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tasks_utils/catkin_generated/installspace/local_setup.sh"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install/setup.zsh;/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install" TYPE FILE FILES
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tasks_utils/catkin_generated/installspace/setup.zsh"
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tasks_utils/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install" TYPE FILE FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tasks_utils/catkin_generated/installspace/.rosinstall")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tasks_utils/catkin_generated/installspace/tasks_utils.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tasks_utils/cmake" TYPE FILE FILES
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tasks_utils/catkin_generated/installspace/tasks_utilsConfig.cmake"
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tasks_utils/catkin_generated/installspace/tasks_utilsConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tasks_utils" TYPE FILE FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/src/tasks_utils/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tasks_utils" TYPE PROGRAM FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tasks_utils/catkin_generated/installspace/check_odom_error.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tasks_utils" TYPE PROGRAM FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tasks_utils/catkin_generated/installspace/firmware_velocity.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tasks_utils" TYPE PROGRAM FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tasks_utils/catkin_generated/installspace/plot_path.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tasks_utils" TYPE PROGRAM FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tasks_utils/catkin_generated/installspace/convert_gt.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tasks_utils" TYPE PROGRAM FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tasks_utils/catkin_generated/installspace/plot_odom_error.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tasks_utils" TYPE PROGRAM FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tasks_utils/catkin_generated/installspace/plot_odom_quality.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tasks_utils" TYPE PROGRAM FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tasks_utils/catkin_generated/installspace/firmware_velocity_LEO.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tasks_utils" TYPE PROGRAM FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tasks_utils/catkin_generated/installspace/firmware_CC8.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tasks_utils" TYPE PROGRAM FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tasks_utils/catkin_generated/installspace/capture_camera.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tasks_utils" TYPE PROGRAM FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tasks_utils/catkin_generated/installspace/input_from_cmd.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tasks_utils" TYPE PROGRAM FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tasks_utils/catkin_generated/installspace/camera_setup.py")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tasks_utils/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tasks_utils/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
