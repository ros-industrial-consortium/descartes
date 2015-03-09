cmake_minimum_required(VERSION 2.8.3)
project(descartes_core)

add_definitions(-std=gnu++0x)
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

