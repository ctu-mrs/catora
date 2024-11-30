cmake_minimum_required(VERSION 2.8.3)
project(mrs_formation_reshaping)

# set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# uncomment and build for debugging
add_definitions(-O3)
# add_compile_options(-w)
# add_definitions(-Og)
# set(CMAKE_WARN_DEPRECATED OFF)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    nodelet
    message_generation
    std_msgs
    sensor_msgs
    geometry_msgs
    std_srvs
    mrs_lib
)

# Include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

# SERVICES
add_message_files(DIRECTORY msg FILES
  TrajectoryMsg.msg
)

# SERVICES
add_service_files(DIRECTORY srv FILES
  SrvGetAssignment.srv
  SrvGetReshapingTrajectories.srv
  SrvReshapeFormation.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES FormationReshaper FormationReshapingNodelet
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs std_srvs mrs_lib message_runtime
  DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  INCLUDE_DIRS include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

# This causes adds the -march=native gcc flag that prevents ditributed compilation
add_definitions(
  )

# Nodelet library FormationController
add_library(FormationReshaper
  src/formation_reshaper.cpp
  src/robot_state.cpp
  src/robot.cpp
  src/type_convertor.cpp
)

# Nodelet library FormationController
add_library(FormationReshapingNodelet
  src/formation_reshaping_nodelet.cpp
  src/formation_reshaper.cpp
  src/robot_state.cpp
  src/robot.cpp
  src/type_convertor.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  FormationReshaper ${catkin_LIBRARIES} ${Eigen_LIBRARIES}
)

target_link_libraries(
  FormationReshapingNodelet ${catkin_LIBRARIES} ${Eigen_LIBRARIES}
)

# target_include_directories(MrsFormationReshaper SYSTEM PRIVATE)

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.xml"
)
## Specify libraries to link a library or executable target against
