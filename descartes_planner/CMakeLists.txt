cmake_minimum_required(VERSION 2.8.3)
project(descartes_planner)
add_definitions(-std=gnu++0x)
find_package(catkin REQUIRED COMPONENTS
  descartes_core
  descartes_trajectory
  console_bridge
  moveit_core
  roscpp
)
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
    test
  LIBRARIES
    descartes_planner
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
add_library(descartes_planner
            src/planning_graph.cpp
            src/sparse_planner.cpp
)

target_link_libraries(descartes_planner
                      ${catkin_LIBRARIES}
)

#############
## Install ##
#############



#############
## Testing ##
#############


if(CATKIN_ENABLE_TESTING)

  set(UTEST_SPARSE_PLANER_SRC_FILES test/utest.cpp
      test/sparse_planner.cpp)
  catkin_add_gtest(${PROJECT_NAME}_sparse_planner_utest ${UTEST_SPARSE_PLANER_SRC_FILES})
  target_link_libraries(${PROJECT_NAME}_sparse_planner_utest descartes_core)

endif()
