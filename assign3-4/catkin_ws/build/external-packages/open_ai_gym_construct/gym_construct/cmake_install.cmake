# Install script for directory: /home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/src/external-packages/open_ai_gym_construct/gym_construct

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/open_ai_gym_construct/gym_construct/catkin_generated/installspace/gym_construct.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gym_construct/cmake" TYPE FILE FILES
    "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/open_ai_gym_construct/gym_construct/catkin_generated/installspace/gym_constructConfig.cmake"
    "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/open_ai_gym_construct/gym_construct/catkin_generated/installspace/gym_constructConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gym_construct" TYPE FILE FILES "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/src/external-packages/open_ai_gym_construct/gym_construct/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gym_construct" TYPE DIRECTORY FILES "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/src/external-packages/open_ai_gym_construct/gym_construct/launch")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gym_construct" TYPE DIRECTORY FILES "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/src/external-packages/open_ai_gym_construct/gym_construct/meshes")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gym_construct" TYPE DIRECTORY FILES "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/src/external-packages/open_ai_gym_construct/gym_construct/models")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gym_construct" TYPE DIRECTORY FILES "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/src/external-packages/open_ai_gym_construct/gym_construct/params")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gym_construct" TYPE DIRECTORY FILES "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/src/external-packages/open_ai_gym_construct/gym_construct/urdf")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gym_construct" TYPE DIRECTORY FILES "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/src/external-packages/open_ai_gym_construct/gym_construct/worlds")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/gym_construct" TYPE PROGRAM FILES "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/open_ai_gym_construct/gym_construct/catkin_generated/installspace/circuit2_turtlebot_lidar_qlearn.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/gym_construct" TYPE PROGRAM FILES "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/open_ai_gym_construct/gym_construct/catkin_generated/installspace/circuit2_turtlebot_lidar_sarsa.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/gym_construct" TYPE PROGRAM FILES "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/open_ai_gym_construct/gym_construct/catkin_generated/installspace/circuit_turtlebot_lidar_qlearn.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/gym_construct" TYPE PROGRAM FILES "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/open_ai_gym_construct/gym_construct/catkin_generated/installspace/circuit_turtlebot_lidar_qlearn_original.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/gym_construct" TYPE PROGRAM FILES "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/open_ai_gym_construct/gym_construct/catkin_generated/installspace/circuit_turtlebot_lidar_qlearn_userspace.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/gym_construct" TYPE PROGRAM FILES "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/open_ai_gym_construct/gym_construct/catkin_generated/installspace/display_plot.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/gym_construct" TYPE PROGRAM FILES "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/open_ai_gym_construct/gym_construct/catkin_generated/installspace/liveplot.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/gym_construct" TYPE PROGRAM FILES "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/open_ai_gym_construct/gym_construct/catkin_generated/installspace/maze_turtlebot_lidar_qlearn.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/gym_construct" TYPE PROGRAM FILES "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/open_ai_gym_construct/gym_construct/catkin_generated/installspace/myenv_turtlebot_lidar_qlearn.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/gym_construct" TYPE PROGRAM FILES "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/open_ai_gym_construct/gym_construct/catkin_generated/installspace/plot_gazebo_gym_experiments.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/gym_construct" TYPE PROGRAM FILES "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/open_ai_gym_construct/gym_construct/catkin_generated/installspace/qlearn.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/gym_construct" TYPE PROGRAM FILES "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/open_ai_gym_construct/gym_construct/catkin_generated/installspace/round_turtlebot_lidar_test.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/gym_construct" TYPE PROGRAM FILES "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/open_ai_gym_construct/gym_construct/catkin_generated/installspace/sarsa.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/gym_construct" TYPE PROGRAM FILES "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/open_ai_gym_construct/gym_construct/catkin_generated/installspace/simple_qlearn.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/gym_construct" TYPE PROGRAM FILES "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/open_ai_gym_construct/gym_construct/catkin_generated/installspace/simple_qlearn_quadcopter.py")
endif()

