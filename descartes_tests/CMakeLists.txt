cmake_minimum_required(VERSION 2.8.3)
project(descartes_tests)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  descartes_moveit
  descartes_planner
  descartes_trajectory
  eigen_conversions
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    descartes_moveit
    descartes_planner
    descartes_trajectory
    eigen_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/cartesian_robot.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

if(CATKIN_ENABLE_TESTING)
  # Trajectory Unit Tests
  catkin_add_gtest(${PROJECT_NAME}_trajectory_utest
    test/trajectory/utest.cpp
    test/trajectory/axial_symmetric_pt.cpp
    test/trajectory/cart_trajectory_pt.cpp
    test/trajectory/joint_trajectory_pt.cpp
    test/trajectory/cartesian_robot_test.cpp
  )
  target_compile_definitions(${PROJECT_NAME}_trajectory_utest PUBLIC GTEST_USE_OWN_TR1_TUPLE=0)
  target_link_libraries(${PROJECT_NAME}_trajectory_utest ${PROJECT_NAME})

  # Planner unit tests
  catkin_add_gtest(${PROJECT_NAME}_planner_utest
    test/planner/utest.cpp
    test/planner/dense_planner.cpp
    test/planner/sparse_planner.cpp
    test/planner/planning_graph_tests.cpp
    test/planner/utils/trajectory_maker.cpp
  )
  target_compile_definitions(${PROJECT_NAME}_planner_utest PUBLIC GTEST_USE_OWN_TR1_TUPLE=0)
  target_link_libraries(${PROJECT_NAME}_planner_utest ${PROJECT_NAME})

  # Moveit adapter unit tests
  find_package(rostest REQUIRED)
  add_rostest_gtest(${PROJECT_NAME}_moveit_utest
    test/moveit/launch/utest.launch
    test/moveit/utest.cpp
    test/moveit/moveit_state_adapter_test.cpp
  )
  target_compile_definitions(${PROJECT_NAME}_moveit_utest PUBLIC GTEST_USE_OWN_TR1_TUPLE=0)
  target_link_libraries(${PROJECT_NAME}_moveit_utest ${PROJECT_NAME})

endif()
