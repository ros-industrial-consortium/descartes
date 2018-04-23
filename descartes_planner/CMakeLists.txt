cmake_minimum_required(VERSION 2.8.11)
project(descartes_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  descartes_core
  descartes_trajectory
  pluginlib
  roscpp
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

# Let's try to use open-mp parallization if we can
find_package(OpenMP)
if (OPENMP_FOUND)
  message(STATUS "Descartes will use OpenMP")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
endif()

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    descartes_core
    descartes_trajectory
    roscpp
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

## DescartesTrajectoryPt lib
add_library(${PROJECT_NAME}
            src/dense_planner.cpp
            src/ladder_graph_dag_search.cpp
            src/planning_graph.cpp
            src/plugin_init.cpp
            src/sparse_planner.cpp
)

target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      ${OpenMP_FLAGS}
)

target_compile_options(${PROJECT_NAME} PRIVATE ${OpenMP_FLAGS})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############
install(
    TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
  FILES ${PROJECT_NAME}_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
