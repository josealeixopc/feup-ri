execute_process(COMMAND "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/turtlebot3-master/turtlebot3_example/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/turtlebot3-master/turtlebot3_example/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
