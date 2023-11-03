# Install script for directory: /home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/src/tag_utils

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
  file(INSTALL DESTINATION "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install" TYPE PROGRAM FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/catkin_generated/installspace/_setup_util.py")
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
  file(INSTALL DESTINATION "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install" TYPE PROGRAM FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/catkin_generated/installspace/env.sh")
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
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/catkin_generated/installspace/setup.bash"
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/catkin_generated/installspace/local_setup.bash"
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
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/catkin_generated/installspace/setup.sh"
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/catkin_generated/installspace/local_setup.sh"
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
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/catkin_generated/installspace/setup.zsh"
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/catkin_generated/installspace/local_setup.zsh"
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
  file(INSTALL DESTINATION "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/install" TYPE FILE FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/catkin_generated/installspace/.rosinstall")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/catkin_generated/safe_execute_install.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tag_utils/srv" TYPE FILE FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/src/tag_utils/srv/GetTag.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tag_utils/cmake" TYPE FILE FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/catkin_generated/installspace/tag_utils-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/devel/.private/tag_utils/include/tag_utils")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/devel/.private/tag_utils/share/roseus/ros/tag_utils")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/devel/.private/tag_utils/share/common-lisp/ros/tag_utils")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/devel/.private/tag_utils/share/gennodejs/ros/tag_utils")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/devel/.private/tag_utils/lib/python3/dist-packages/tag_utils")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/devel/.private/tag_utils/lib/python3/dist-packages/tag_utils" REGEX "/\\_\\_init\\_\\_\\.py$" EXCLUDE REGEX "/\\_\\_init\\_\\_\\.pyc$" EXCLUDE)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/devel/.private/tag_utils/lib/python3/dist-packages/tag_utils" FILES_MATCHING REGEX "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/devel/\\.private/tag_utils/lib/python3/dist-packages/tag_utils/.+/__init__.pyc?$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/catkin_generated/installspace/tag_utils.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tag_utils/cmake" TYPE FILE FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/catkin_generated/installspace/tag_utils-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tag_utils/cmake" TYPE FILE FILES
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/catkin_generated/installspace/tag_utilsConfig.cmake"
    "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/catkin_generated/installspace/tag_utilsConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tag_utils" TYPE FILE FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/src/tag_utils/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tag_utils" TYPE PROGRAM FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/catkin_generated/installspace/detect_tag.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tag_utils" TYPE PROGRAM FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/catkin_generated/installspace/ekf_tag.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tag_utils" TYPE PROGRAM FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/catkin_generated/installspace/maintenance_tag_detection.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tag_utils" TYPE PROGRAM FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/catkin_generated/installspace/maintenance_tag_detection_simple.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tag_utils" TYPE PROGRAM FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/catkin_generated/installspace/navigation_tag_detection.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tag_utils" TYPE PROGRAM FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/catkin_generated/installspace/maintenance_buttons_site.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tag_utils" TYPE PROGRAM FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/catkin_generated/installspace/maintenance_presa.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tag_utils" TYPE PROGRAM FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/catkin_generated/installspace/remote_objective_1.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tag_utils" TYPE PROGRAM FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/catkin_generated/installspace/remote_objective_2.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tag_utils" TYPE PROGRAM FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/catkin_generated/installspace/remote_objective_3.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tag_utils" TYPE PROGRAM FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/catkin_generated/installspace/remote_objective_9.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tag_utils" TYPE PROGRAM FILES "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/catkin_generated/installspace/service_get_tag.py")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
