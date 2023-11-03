execute_process(COMMAND "/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/toto/Documents/SASA/SASA/Robotic-Arm/rob_arm_ws/build/tag_utils/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
