# Install script for directory: /home/ryven/D-Star-Lite-Implementation-on-Turtlebot3-Waffle-/D_star_lite/src/waffle_lab

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ryven/D-Star-Lite-Implementation-on-Turtlebot3-Waffle-/D_star_lite/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/waffle_lab/msg" TYPE FILE FILES "/home/ryven/D-Star-Lite-Implementation-on-Turtlebot3-Waffle-/D_star_lite/src/waffle_lab/msg/my_msg.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/waffle_lab/cmake" TYPE FILE FILES "/home/ryven/D-Star-Lite-Implementation-on-Turtlebot3-Waffle-/D_star_lite/src/waffle_lab/catkin_generated/installspace/waffle_lab-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/ryven/D-Star-Lite-Implementation-on-Turtlebot3-Waffle-/D_star_lite/devel/include/waffle_lab")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/ryven/D-Star-Lite-Implementation-on-Turtlebot3-Waffle-/D_star_lite/devel/share/roseus/ros/waffle_lab")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/ryven/D-Star-Lite-Implementation-on-Turtlebot3-Waffle-/D_star_lite/devel/share/common-lisp/ros/waffle_lab")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/ryven/D-Star-Lite-Implementation-on-Turtlebot3-Waffle-/D_star_lite/devel/share/gennodejs/ros/waffle_lab")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/ryven/D-Star-Lite-Implementation-on-Turtlebot3-Waffle-/D_star_lite/devel/lib/python3/dist-packages/waffle_lab")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/ryven/D-Star-Lite-Implementation-on-Turtlebot3-Waffle-/D_star_lite/devel/lib/python3/dist-packages/waffle_lab")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ryven/D-Star-Lite-Implementation-on-Turtlebot3-Waffle-/D_star_lite/src/waffle_lab/catkin_generated/installspace/waffle_lab.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/waffle_lab/cmake" TYPE FILE FILES "/home/ryven/D-Star-Lite-Implementation-on-Turtlebot3-Waffle-/D_star_lite/src/waffle_lab/catkin_generated/installspace/waffle_lab-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/waffle_lab/cmake" TYPE FILE FILES
    "/home/ryven/D-Star-Lite-Implementation-on-Turtlebot3-Waffle-/D_star_lite/src/waffle_lab/catkin_generated/installspace/waffle_labConfig.cmake"
    "/home/ryven/D-Star-Lite-Implementation-on-Turtlebot3-Waffle-/D_star_lite/src/waffle_lab/catkin_generated/installspace/waffle_labConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/waffle_lab" TYPE FILE FILES "/home/ryven/D-Star-Lite-Implementation-on-Turtlebot3-Waffle-/D_star_lite/src/waffle_lab/package.xml")
endif()

