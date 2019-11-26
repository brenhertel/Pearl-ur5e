# Install script for directory: /home/bhertel/catkin_ws/src/robotiq/robotiq_2f_c2_gripper_visualization

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/bhertel/catkin_ws/install")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/bhertel/catkin_ws/build/robotiq/robotiq_2f_c2_gripper_visualization/catkin_generated/installspace/robotiq_2f_c2_gripper_visualization.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_2f_c2_gripper_visualization/cmake" TYPE FILE FILES
    "/home/bhertel/catkin_ws/build/robotiq/robotiq_2f_c2_gripper_visualization/catkin_generated/installspace/robotiq_2f_c2_gripper_visualizationConfig.cmake"
    "/home/bhertel/catkin_ws/build/robotiq/robotiq_2f_c2_gripper_visualization/catkin_generated/installspace/robotiq_2f_c2_gripper_visualizationConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_2f_c2_gripper_visualization" TYPE FILE FILES "/home/bhertel/catkin_ws/src/robotiq/robotiq_2f_c2_gripper_visualization/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_2f_c2_gripper_visualization" TYPE DIRECTORY FILES "/home/bhertel/catkin_ws/src/robotiq/robotiq_2f_c2_gripper_visualization/urdf")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_2f_c2_gripper_visualization" TYPE DIRECTORY FILES "/home/bhertel/catkin_ws/src/robotiq/robotiq_2f_c2_gripper_visualization/meshes")
endif()

