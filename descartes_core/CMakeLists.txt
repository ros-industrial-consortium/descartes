cmake_minimum_required(VERSION 2.8.3)
project(descartes_core)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  console_bridge
  moveit_core
  roscpp
)

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)


catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    descartes_core
  CATKIN_DEPENDS
    roscpp
    moveit_core
  DEPENDS
    Boost
    console_bridge
    Eigen
)

###########
## Build ##
###########
include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${Eigen_INCLUDE_DIRS}
)

## DescartesTrajectoryPt lib
add_library(descartes_core
            src/trajectory_pt_transition.cpp
            src/cart_trajectory_pt.cpp
            src/joint_trajectory_pt.cpp
            src/trajectory.cpp
            src/planning_graph.cpp
)
##Disabling planning graph due to issues with
##Trajectory point ID
##  src/planning_graph.cpp
target_link_libraries(descartes_core
                      ${catkin_LIBRARIES}
)


#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  set(UTEST_SRC_FILES test/utest.cpp
      test/trajectory_pt.cpp
      test/cart_trajectory_pt.cpp
      test/joint_trajectory_pt.cpp)
  catkin_add_gtest(utest ${UTEST_SRC_FILES})
  target_link_libraries(utest descartes_core)
endif()
