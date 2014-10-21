cmake_minimum_required(VERSION 2.8.3)
project(descartes_moveit)

find_package(catkin REQUIRED COMPONENTS
  console_bridge
  moveit_core
  tf
  descartes_trajectory_planning
)

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)


catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    descartes_moveit
  CATKIN_DEPENDS
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

## Descartes MoveIt lib
add_library(descartes_moveit
            src/moveit_state_adapter.cpp
)
target_link_libraries(descartes_moveit
                      ${catkin_LIBRARIES}
)


#############
## Testing ##
#############

# catkin_add_gtest(${PROJECT_NAME}-test test/test_descartes_trajectory_planning.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
