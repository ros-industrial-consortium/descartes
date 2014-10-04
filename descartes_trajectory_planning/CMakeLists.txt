cmake_minimum_required(VERSION 2.8.3)
project(descartes_trajectory_planning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
)

find_package(Boost REQUIRED)
message("Boost libraries:" ${Boost_LIBRARIES})
message("Boost includes:" ${Boost_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
message("Eigen libraries:" ${Eigen_LIBRARIES})
message("Eigen includes:" ${Eigen_INCLUDE_DIRS})


catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    trajectory_pt_transition
    trajectory_pt
    cart_trajectory_pt
    joint_trajectory_pt
    trajectory
  CATKIN_DEPENDS
    roscpp
    moveit_core
  DEPENDS
    Boost
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

## TrajectoryTransition lib
add_library(trajectory_pt_transition
            src/trajectory_pt_transition.cpp
)
target_link_libraries(trajectory_pt_transition
                      ${catkin_LIBRARIES}
)

## TrajectoryPt lib
add_library(trajectory_pt
            src/trajectory_pt.cpp
)
target_link_libraries(trajectory_pt
                      trajectory_pt_transition
                      ${catkin_LIBRARIES}
)

## CartTrajectoryPt lib
add_library(cart_trajectory_pt
            src/cart_trajectory_pt.cpp
)
target_link_libraries(cart_trajectory_pt
                      trajectory_pt
                      ${catkin_LIBRARIES}
)

## JointTrajectoryPt lib
add_library(joint_trajectory_pt
            src/joint_trajectory_pt.cpp
)
target_link_libraries(joint_trajectory_pt
                      trajectory_pt
                      ${catkin_LIBRARIES}
)

## Trajectory lib
add_library(trajectory
            src/trajectory.cpp
)
target_link_libraries(trajectory
                      trajectory_pt
                      ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

# catkin_add_gtest(${PROJECT_NAME}-test test/test_descartes_trajectory_planning.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
