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
#include "descartes_trajectory_test/cartesian_robot.h"
#include <gtest/gtest.h>

using namespace descartes_core;
using namespace descartes_trajectory;
using namespace descartes_trajectory_test;

const static double TOOL_SPEED = 0.5f; // m/s
const static double ZONE_RADIUS = 2.0f; // meters
const static double INTERPOLATION_INTERVAL = 0.5f; // secs
const static double POINT_DISTANCE = 20.0f;
const static int NUM_POINTS = 5;
const double POS_RANGE = 2.0;
const double ORIENT_RANGE = 1.0;


void createSimpleTrajectory(std::vector<TrajectoryPtPtr>& traj)
{

  double angular_z_step = M_PI/2;
  tf::Vector3 linear_step = tf::Vector3(0,POINT_DISTANCE,0);
  tf::Transform pose = tf::Transform::getIdentity();
  Eigen::Affine3d eigen_pose;

  traj.resize(NUM_POINTS);
  for(int i = 0; i < NUM_POINTS ; i++)
  {
    pose = pose * tf::Transform(tf::Quaternion(tf::Vector3(0,0,1),angular_z_step))*
        tf::Transform(tf::Quaternion::getIdentity(),linear_step);
    tf::poseTFToEigen(pose,eigen_pose);
    traj[i] = TrajectoryPtPtr(new CartTrajectoryPt(eigen_pose));
  }
}

TEST(CartesianInterpolator,interpolate)
{
  descartes_trajectory::CartesianInterpolator c;
  std::vector<TrajectoryPtPtr> coarse_traj;
  std::vector<descartes_core::TrajectoryPtPtr> interpolated_traj;
  RobotModelConstPtr robot(new CartesianRobot(POS_RANGE, ORIENT_RANGE,6));

  // initializing interpolator
  EXPECT_TRUE(c.initialize(robot,TOOL_SPEED,INTERPOLATION_INTERVAL,ZONE_RADIUS));

  // interpolating
  double time_per_segment = POINT_DISTANCE/TOOL_SPEED;
  int points_per_segment = (time_per_segment/INTERPOLATION_INTERVAL) + 1;
  int num_segments = NUM_POINTS - 1;
  int expected_points = (points_per_segment) * (num_segments) - (num_segments - 1);
  createSimpleTrajectory(coarse_traj);
  EXPECT_TRUE(c.interpolate(coarse_traj,interpolated_traj));

  // expected point count
  EXPECT_EQ(expected_points,interpolated_traj.size());
  std::cout<<"INTERPOLATION YIELDED "<<interpolated_traj.size()<<" POINTS"<<
      " FROM A COARSE TRAJECTORY WITH "<<NUM_POINTS<<" POINTS." <<std::endl;
}









