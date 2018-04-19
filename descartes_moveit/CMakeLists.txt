cmake_minimum_required(VERSION 2.8.11)
project(descartes_moveit)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  descartes_core
  descartes_trajectory
  moveit_core
  moveit_ros_planning
  pluginlib
  rosconsole_bridge
  tf
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    cmake_modules
    descartes_core
    descartes_trajectory
    moveit_core
    rosconsole_bridge
    tf
  DEPENDS
    Boost
    EIGEN3
)

###########
## Build ##
###########
include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
)

## Descartes MoveIt lib
add_library(${PROJECT_NAME}
            src/ikfast_moveit_state_adapter.cpp
            src/moveit_state_adapter.cpp
            src/plugin_init.cpp
            src/seed_search.cpp
)
target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(
    TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
  FILES moveit_adapter_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
