#include "descartes_trajectory_test/cartesian_robot.h"
#include "descartes_trajectory_test/robot_model_test.hpp"
#include "descartes_planner/sparse_planner.h"
#include "descartes_planner/planning_graph.h"

using namespace descartes_core;
using namespace descartes_planner;

using testing::Types;

namespace descartes_planner_test
{

descartes_planner::SparsePlanner SparsePlanner;
descartes_planner::DensePlanner DensePlanner;

template<class T>
class SparsePlannerTest : public descartes_planner_test::DescartesPlannerTest<T>{};

template<class T>
class DensePlannerTest : public descartes_planner_test::DescartesPlannerTest<T>{};


INSTANTIATE_TYPED_TEST_CASE_P(SparsePlannerTest, DescartesPlannerTest, SparsePlanner);
INSTANTIATE_TYPED_TEST_CASE_P(DensePlannerTest, DescartesPlannerTest, DensePlanner);

} //descartes_planner_test