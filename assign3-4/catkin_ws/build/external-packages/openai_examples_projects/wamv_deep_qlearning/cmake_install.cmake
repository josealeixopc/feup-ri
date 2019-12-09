# Install script for directory: /home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/src/external-packages/openai_examples_projects/wamv_deep_qlearning

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/openai_examples_projects/wamv_deep_qlearning/catkin_generated/installspace/wamv_deep_qlearning.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/wamv_deep_qlearning/cmake" TYPE FILE FILES
    "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/openai_examples_projects/wamv_deep_qlearning/catkin_generated/installspace/wamv_deep_qlearningConfig.cmake"
    "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/openai_examples_projects/wamv_deep_qlearning/catkin_generated/installspace/wamv_deep_qlearningConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/wamv_deep_qlearning" TYPE FILE FILES "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/src/external-packages/openai_examples_projects/wamv_deep_qlearning/package.xml")
endif()

