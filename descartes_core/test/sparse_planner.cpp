#include <descartes_core/sparse_planner.h>
#include "descartes_core/cart_trajectory_pt.h"
#include "descartes_core/utils.h"
#include "descartes_core_test/cartesian_robot.h"
#include <gtest/gtest.h>
#include <tuple>

using namespace descartes_core;
typedef std::vector<descartes_core::TrajectoryPtPtr> Trajectory;
const int NUM_DENSE_POINTS = 1000;
Trajectory createTestTrajectory();
Trajectory TEST_TRAJECTORY = createTestTrajectory();

class TestPoint: public descartes_core::CartTrajectoryPt
{
public:
  TestPoint(const std::vector<double>& joints)
  {
    vals_.resize(joints.size());
    vals_.assign(joints.begin(),joints.end());
  }

  virtual ~TestPoint()
  {

  }

  virtual bool getClosestJointPose(const std::vector<double> &seed_state,
                                     const RobotModel &model,
                                     std::vector<double> &joint_pose) const
  {
    joint_pose.clear();
    joint_pose.assign(vals_.begin(),vals_.end());
    return true;
  }

protected:

  std::vector<double> vals_;
};

Trajectory createTestTrajectory()
{
  ROS_INFO_STREAM("Creating test trajectory with "<<NUM_DENSE_POINTS<<" points");
  Trajectory traj;
  std::vector<std::tuple<double, double>>joint_bounds = {std::make_tuple(0,M_PI),
                                                         std::make_tuple(-M_PI_2,M_PI_2),
                                                         std::make_tuple(M_PI/8,M_PI/3)};
  std::vector<double> deltas;
  for(auto& e:joint_bounds)
  {
    double d = (std::get<1>(e)- std::get<0>(e))/NUM_DENSE_POINTS;
    deltas.push_back(d);
  }

  // creating trajectory points
  std::vector<double> joint_vals(deltas.size(),0);
  traj.reserve(NUM_DENSE_POINTS);
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
  RobotModelConstPtr robot(new descartes_core_test::CartesianRobot(0,0));
  descartes_core::SparsePlanner planner(robot);

  ROS_INFO_STREAM("Testing setTrajectoryPoints() with "<<NUM_DENSE_POINTS<<" dense points");
  EXPECT_TRUE(planner.setPoints(TEST_TRAJECTORY));
}
