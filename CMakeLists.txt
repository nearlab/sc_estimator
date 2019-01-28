cmake_minimum_required(VERSION 3.0)
project(sc_estimator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  nearlab_msgs
  nearlab_utils
  roscpp
)

find_library(NL_UTILS NAMES nearlab_utils)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
#set(EIGEN3_INCLUDE_DIRS /usr/share/eigen)

catkin_package(CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  nearlab_msgs
  nearlab_utils
  roscpp
  message_runtime
)

#########
# BUILD #
#########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(sc_estimator_node
               src/scEstimatorNode.cpp
               src/estimator.cpp
               src/simulator.cpp)

set(EXEC_LIST sc_estimator_node)
foreach(EXEC_FILE ${EXEC_LIST})
  target_link_libraries(${EXEC_FILE} ${catkin_LIBRARIES} ${NL_UTILS})
  install(TARGETS ${EXEC_FILE}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach(EXEC_FILE ${EXEC_LIST})
#install(DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#  PATTERN ".svn" EXCLUDE
#)
