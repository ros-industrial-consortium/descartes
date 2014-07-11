cmake_minimum_required(VERSION 2.8.3)
project(descartes_path_planning)

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
#  LIBRARIES descartes_path_planning
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

## ProcessPt lib
add_library(descartes_process_pt
            src/process_pt.cpp
)
target_link_libraries(descartes_process_pt
                      ${catkin_LIBRARIES}
)

## ProcessTransition lib
add_library(descartes_process_transition
            src/process_transition.cpp
)
target_link_libraries(descartes_process_transition
                      ${catkin_LIBRARIES}
)

## ProcessPath lib
add_library(descartes_process_path
            src/process_path.cpp
)
target_link_libraries(descartes_process_path
                      ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

# catkin_add_gtest(${PROJECT_NAME}-test test/test_descartes_path_planning.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
