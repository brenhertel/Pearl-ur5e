# Install script for directory: /home/bhertel/catkin_ws/src/robotiq/robotiq_3f_gripper_joint_state_publisher

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/bhertel/catkin_ws/build/robotiq/robotiq_3f_gripper_joint_state_publisher/catkin_generated/installspace/robotiq_3f_gripper_joint_state_publisher.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_3f_gripper_joint_state_publisher/cmake" TYPE FILE FILES
    "/home/bhertel/catkin_ws/build/robotiq/robotiq_3f_gripper_joint_state_publisher/catkin_generated/installspace/robotiq_3f_gripper_joint_state_publisherConfig.cmake"
    "/home/bhertel/catkin_ws/build/robotiq/robotiq_3f_gripper_joint_state_publisher/catkin_generated/installspace/robotiq_3f_gripper_joint_state_publisherConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_3f_gripper_joint_state_publisher" TYPE FILE FILES "/home/bhertel/catkin_ws/src/robotiq/robotiq_3f_gripper_joint_state_publisher/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robotiq_3f_gripper_joint_state_publisher/robotiq_3f_gripper_joint_states" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robotiq_3f_gripper_joint_state_publisher/robotiq_3f_gripper_joint_states")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robotiq_3f_gripper_joint_state_publisher/robotiq_3f_gripper_joint_states"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robotiq_3f_gripper_joint_state_publisher" TYPE EXECUTABLE FILES "/home/bhertel/catkin_ws/devel/lib/robotiq_3f_gripper_joint_state_publisher/robotiq_3f_gripper_joint_states")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robotiq_3f_gripper_joint_state_publisher/robotiq_3f_gripper_joint_states" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robotiq_3f_gripper_joint_state_publisher/robotiq_3f_gripper_joint_states")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robotiq_3f_gripper_joint_state_publisher/robotiq_3f_gripper_joint_states"
         OLD_RPATH "/home/bhertel/catkin_ws/devel/lib:/opt/ros/kinetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robotiq_3f_gripper_joint_state_publisher/robotiq_3f_gripper_joint_states")
    endif()
  endif()
endif()

