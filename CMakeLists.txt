cmake_minimum_required(VERSION 3.0)
project(sc_estimator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  nearlab_msgs
  roscpp
)

#find_package(Eigen3 3.3 REQUIRED NO_MODULE)
set(EIGEN3_INCLUDE_DIRS /usr/share/eigen)

catkin_package(CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  nearlab_msgs
  roscpp
  message_runtime
)

#########
# BUILD #
#########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

add_executable(sc_estimator_node
               src/scEstimatorNode.cpp
               src/estimator.cpp
               src/simulator.cpp)

set(EXEC_LIST sc_estimator_node)
foreach(EXEC_FILE ${EXEC_LIST})
  target_link_libraries(${EXEC_FILE} ${catkin_LIBRARIES} Eigen3::Eigen)
  add_dependencies(${EXEC_FILE} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
  install(TARGETS ${EXEC_FILE}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach(EXEC_FILE ${EXEC_LIST})
#install(DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#  PATTERN ".svn" EXCLUDE
#)
