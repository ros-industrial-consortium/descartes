#include "descartes_trajectory_test/cartesian_robot.h"
#include "descartes_trajectory_test/robot_model_test.hpp"
#include "descartes_planner/sparse_planner.h"
#include "descartes_planner/planning_graph.h"

using namespace descartes_core;

using testing::Types;

namespace descartes_planner_test
{

template <>
RobotModelPtr CreateRobotModel<CartesianRobot>()
{
  return RobotModelPtr(new CartesianRobot());
}

template<class T>
class CartesianRobotModelTest : public descartes_trajectory_test::RobotModelTest<T>{};

INSTANTIATE_TYPED_TEST_CASE_P(DescartesPlannerTest, SparsePlanner);

} //descartes_trajectory_test
