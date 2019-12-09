# Install script for directory: /home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/install")
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
   "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/install" TYPE PROGRAM FILES "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/install" TYPE PROGRAM FILES "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/install/setup.bash;/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/install" TYPE FILE FILES
    "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/catkin_generated/installspace/setup.bash"
    "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/install/setup.sh;/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/install" TYPE FILE FILES
    "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/catkin_generated/installspace/setup.sh"
    "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/install/setup.zsh;/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/install" TYPE FILE FILES
    "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/catkin_generated/installspace/setup.zsh"
    "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/install" TYPE FILE FILES "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/gtest/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/turtlebot3-master/turtlebot3/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/turtlebot3_msgs-master/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/turtlebot3-master/turtlebot3_navigation/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/turtlebot3_simulations-master/turtlebot3_simulations/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/openai_examples_projects/cartpole_openai_ros_examples/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/internal-packages/coop_mapping/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/open_ai_gym_construct/gym_construct/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/openai_examples_projects/moving_cube_training/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/openai_ros/openai_ros/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/openai_examples_projects/fetch_openai_ros_example/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/openai_examples_projects/husarion_openai_ros_example/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/openai_examples_projects/iriwam_openai_ros_example/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/openai_examples_projects/my_hopper_openai_example/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/openai_examples_projects/my_moving_cube_pkg/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/openai_examples_projects/my_parrotdrone_openai_example/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/openai_examples_projects/my_sawyer_openai_example/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/openai_examples_projects/my_shadow_tc_openai_example/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/openai_examples_projects/my_sumitxl_openai_example/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/openai_examples_projects/my_turtlebot3_openai_example/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/m-explore-master/map_merge/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/openai_examples_projects/turtle2_openai_ros_example/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/turtlebot3-master/turtlebot3_bringup/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/turtlebot3-master/turtlebot3_example/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/turtlebot3_simulations-master/turtlebot3_fake/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/turtlebot3_simulations-master/turtlebot3_gazebo/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/turtlebot3-master/turtlebot3_slam/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/turtlebot3-master/turtlebot3_teleop/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/m-explore-master/explore/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/openai_examples_projects/wamv_deep_qlearning/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/openai_examples_projects/wamv_openai_ros_example/cmake_install.cmake")
  include("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/turtlebot3-master/turtlebot3_description/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
