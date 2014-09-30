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
#  INCLUDE_DIRS include
#  LIBRARIES descartes_trajectory_planning
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${Eigen_INCLUDE_DIRS}
)

## TrajectoryPt lib
add_library(descartes_trajectory_pt
            src/trajectory_pt.cpp
)
target_link_libraries(descartes_trajectory_pt
                      ${catkin_LIBRARIES}
)

## TrajectoryTransition lib
add_library(descartes_trajectory_transition
            src/trajectory_transition.cpp
)
target_link_libraries(descartes_trajectory_transition
                      ${catkin_LIBRARIES}
)

## Trajectory lib
add_library(descartes_trajectory
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
