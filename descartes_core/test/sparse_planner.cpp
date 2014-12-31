#include <descartes_core/sparse_planner.h>
#include "descartes_core/cart_trajectory_pt.h"
#include "descartes_core/utils.h"
#include "descartes_core_test/cartesian_robot.h"
#include <gtest/gtest.h>
#include <tuple>

using namespace descartes_core;

typedef std::vector<descartes_core::TrajectoryPtPtr> Trajectory;
const int NUM_DENSE_POINTS = 200;
Trajectory TEST_TRAJECTORY = createTestTrajectory();

class TestPoint: public descartes_core::CartTrajectoryPt
{
public:
  TestPoint(const std::vector<double>& joints)
  {
    vals_.resize(joints.size());
    vals_.assign(vals_.begin(),joints.begin(),joints.end());
  }

  virtual ~TestPoint()
  {

  }

  virtual bool getClosestJointPose(const std::vector<double> &seed_state,
                                     const RobotModel &model,
                                     std::vector<double> &joint_pose) const
  {
    joint_pose.clear();
    joint_pose.assigns(vals.begin(),vals.end());
    return true;
  }

protected:

  std::vector<double> vals_;
}

Trajectory createTestTrajectory()
{
  Trajectory traj;
  std::vector<std::tuple<double, double>>joint_bounds = {std::make_tuple(0,M_PI),
                                                         std::make_tuple(-M_PI_2,M_PI_2),
                                                         std::make_tuple(M_PI/8,M_PI/3)};
  std::vector<double> deltas;
  for(auto& e:joint_bounds)
  {
    double d = (std::get<1>(e) std::get<0>(e))/NUM_DENSE_POINTS;
    deltas.push_back(d);
  }

  // creating trajectory points
  std::vector<double> joint_vals(deltas.size(),0);
  for(int i = 0 ; i < NUM_DENSE_POINTS; i++)
  {
    for(int j = 0; j < deltas.size(); j++)
    {
      joint_vals[j] = std::get<0>(joint_bounds[j]) + deltas[j]*i;
    }
    TrajectoryPtPtr tp(new TestPoint(joint_vals));
    traj.push_back(tp);
  }
  return traj;
}


TEST(SparsePlanner, setTrajectoryPoints)
{
  descartes_core_test::CartesianRobot robot(0, 0);
  descartes_core::SparsePlanner planner(robot);
  EXPECT_TRUE(planner.setTrajectoryPoints(TEST_TRAJECTORY));
}
