/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <descartes_trajectory/cartesian_interpolator.h>
#include <tf_conversions/tf_eigen.h>
#include <descartes_trajectory_test/cartesian_robot.h>
#include <gtest/gtest.h>

using namespace descartes_core;
using namespace descartes_trajectory;
using namespace descartes_trajectory_test;

const static double TOOL_SPEED = 0.01f; // m/s
const static double ZONE_RADIUS = 0.01f; // meters
const static double INTERPOLATION_INTERVAL = 0.1f; // secs
const double POS_RANGE = 2.0;
const double ORIENT_RANGE = 1.0;


void createCoarseTrajectory(std::vector<CartTrajectoryPt>& traj)
{
  double a = 0.2f; // amplitude
  double r = 0.4f;
  double thetaf = 2*M_PI;
  double theta0 = 0;
  std::size_t angular_steps = 10;
  double delta_theta = (thetaf - theta0)/(angular_steps - 1);
  std::vector<double> theta_vals(angular_steps,0.0f);
  traj.resize(angular_steps);

  tf::Transform pose;
  Eigen::Affine3d eigen_pose;
  tf::Transform tz = tf::Transform(tf::Quaternion::getIdentity(),tf::Vector3(r,0,0));
  for(int t = 0; t < angular_steps; t++)
  {
    double theta = theta0 + t*delta_theta;
    tf::Transform rz = tf::Transform(tf::Quaternion(tf::Vector3(0,0,1),theta),tf::Vector3(0,0,0));
    tf::Vector3 plocal = tf::Vector3(0,0,a*std::sin((1/r)*theta));
    pose = rz*tz*tf::Transform(tf::Quaternion::getIdentity(),plocal);
    tf::poseTFToEigen(pose,eigen_pose);
    traj[t] = CartTrajectoryPt(eigen_pose);
  }
}

TEST(CartesianInterpolator,interpolate)
{
  descartes_trajectory::CartesianInterpolator c;
  std::vector<CartTrajectoryPt> coarse_traj;
  std::vector<descartes_core::TrajectorPtPtr> interpolated_traj;
  RobotModelConstPtr robot(new CartesianRobot(POS_RANGE, ORIENT_RANGE,6));

  // initializing interpolator
  EXPECT_TRUE(c.initialize(robot,TOOL_SPEED,INTERPOLATION_INTERVAL,ZONE_RADIUS));

  // interpolating
  createCoarseTrajectory(coarse_traj);
  EXPECT_TRUE(c.interpolate(coarse_tra,dense_traj));
}









