# This file is part of the ROS PLUGIN for V-REP
# 
# Copyright 2006-2016 Coppelia Robotics GmbH. All rights reserved. 
# marc@coppeliarobotics.com
# www.coppeliarobotics.com
# 
# Copyright 2017 Jonathan Cacace. All rights reserved. 
# jonathan.cacace@gmail.com
# 
# 
# The ROS PLUGIN is licensed under the terms of GNU GPL:
# 
# -------------------------------------------------------------------
# The ROS PLUGIN is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# THE ROS PLUGIN IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
# WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
# AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
# DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
# MISUSING THIS SOFTWARE.
# 
# See the GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with the ROS PLUGIN.  If not, see <http:#www.gnu.org/licenses/>.
# -------------------------------------------------------------------
#
# This file was updated for Coppeliasim release V4.0.0 

cmake_minimum_required(VERSION 2.8.3)
project(coppeliasim_ros_services)

find_package(catkin REQUIRED COMPONENTS
  coppeliasim_msgs_srvs
  image_transport
  roscpp
  tf
)

find_package( OpenCV REQUIRED )
catkin_package(
               CATKIN_DEPENDS roscpp rosconsole std_msgs rosgraph_msgs sensor_msgs cv_bridge image_transport tf brics_actuator roslib
               DEPENDS roscpp rosconsole std_msgs rosgraph_msgs sensor_msgs cv_bridge image_transport tf brics_actuator roslib)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

link_directories("/opt/ros/melodic/lib")

add_library(simExtRosServices
  include/simLib/simLib.cpp 
  include/simLib/luaFunctionData.cpp 
  include/simLib/luaFunctionDataItem.cpp 

  src/simExtRosServer.cpp 
  src/simExtRosSubscriber.cpp
  src/simExtRosService_plugin.cpp
  )

add_dependencies(simExtRosServices ${catkin_EXPORTED_TARGETS})
target_link_libraries(simExtRosServices
  pthread
  dl
  roslib
  rosconsole
  rostime
  rospack
  roscpp_serialization
  roscpp
  tf
  image_transport
  boost_system
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  )
