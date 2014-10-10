cmake_minimum_required(VERSION 2.8.3)
project(descartes_trajectory_planning)

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
    descartes_trajectory
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
add_library(descartes_trajectory
            src/trajectory_pt_transition.cpp
            src/cart_trajectory_pt.cpp
            src/joint_trajectory_pt.cpp
            src/trajectory.cpp
)
target_link_libraries(descartes_trajectory
                      ${catkin_LIBRARIES}
)


#############
## Testing ##
#############

# catkin_add_gtest(${PROJECT_NAME}-test test/test_descartes_trajectory_planning.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
