cmake_minimum_required(VERSION 2.8.3)
project(auv8_mission_controller_flexbe_states)

#===============================================================================
# C A T K I N   C O N F I G U R A T I O N

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  geometry_msgs
  dynamic_reconfigure
)

#===============================================================================
# G E N E R A T E   M E S S A G E S   &   S E R V I C E S

add_service_files(DIRECTORY srv
  FILES 
  ListMissions.srv
  LoadMission.srv
  StartMission.srv
  StopMission.srv
  CurrentMission.srv
  ReceivedMission.srv
  ReceivedState.srv
  SendMission.srv
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS roscpp
                 rospy
                 geometry_msgs
                 message_runtime
                 dynamic_reconfigure
)

# specify catkin-specific information
# INCLUDE_DIRS - The exported include paths (i.e. cflags) for the package
# LIBRARIES - The exported libraries from the project
# CATKIN_DEPENDS - Other catkin projects that this project depends on
# DEPENDS - Non-catkin CMake projects that this project depends on
# CFG_EXTRAS - Additional configuration options 
catkin_package(
    INCLUDE_DIRS src
    LIBRARIES ${PROJECT_NAME})

# use add_library() or add_executable() as required
#add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})

# install executables
#install(PROGRAMS bin/hello
#        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})