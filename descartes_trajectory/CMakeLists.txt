cmake_minimum_required(VERSION 2.8.11)
project(descartes_trajectory)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  descartes_core
  moveit_core
  roscpp
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

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
    test
  LIBRARIES
    descartes_trajectory
  CATKIN_DEPENDS
    roscpp
    moveit_core
    descartes_core
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

add_library(descartes_trajectory
            src/cart_trajectory_pt.cpp
            src/joint_trajectory_pt.cpp
            src/trajectory.cpp
            src/axial_symmetric_pt.cpp
            test/cartesian_robot.cpp
)

target_link_libraries(descartes_trajectory
                      ${catkin_LIBRARIES}
)


#############
## Install ##
#############

install(
    TARGETS ${PROJECT_NAME} 
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  set(UTEST_SRC_FILES test/utest.cpp
      test/trajectory_pt.cpp
      test/cart_trajectory_pt.cpp
      test/joint_trajectory_pt.cpp
      test/cartesian_robot.cpp
      test/cartesian_robot_test.cpp
      test/axial_symmetric_pt.cpp)

  catkin_add_gtest(${PROJECT_NAME}_utest ${UTEST_SRC_FILES})
  target_compile_definitions(${PROJECT_NAME}_utest PUBLIC GTEST_USE_OWN_TR1_TUPLE=0)
  target_link_libraries(${PROJECT_NAME}_utest descartes_trajectory)

endif()
