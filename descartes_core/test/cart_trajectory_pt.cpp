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

#include "descartes_core/cart_trajectory_pt.h"
#include "descartes_core_test/cartesian_robot.h"
#include <gtest/gtest.h>


using namespace descartes_core;
using namespace descartes_core_test;



TEST(CartTrajPt, construction)
{
CartTrajectoryPt def();
}

TEST(CartTrajPt, getPoses)
{
  const double POS_TOL = 0.5;
  const double POS_INC = 0.1;

  const double ORIENT_TOL = 1.0;
  const double ORIENT_INC = 0.2;

  const double NUM_SAMPLED_POS = pow((POS_TOL/POS_INC) + 1, 3.0);
  const double NUM_SAMPLED_ORIENT = pow((ORIENT_TOL/ORIENT_INC) + 1, 3.0);
  const double NUM_SAMPLED_BOTH = NUM_SAMPLED_POS * NUM_SAMPLED_ORIENT;

  ROS_INFO_STREAM("Expected samples, position: " << NUM_SAMPLED_POS
                  << ", orientation: " << NUM_SAMPLED_ORIENT
                  << ", both: " << NUM_SAMPLED_BOTH);

  ROS_INFO_STREAM("Initializing fuzzy position point");
  CartTrajectoryPt fuzzy_pos(TolerancedFrame( PositionTolerance(0.0, 0.0, 0.0, POS_TOL),
                                       OrientationTolerance(0.0, 0.0, 0.0, 0.0)),
                      POS_INC, ORIENT_INC);

  ROS_INFO_STREAM("Initializing fuzzy orientation point");
  CartTrajectoryPt fuzzy_orient(TolerancedFrame( PositionTolerance(0.0, 0.0, 0.0, 0.0),
                                       OrientationTolerance(0.0, 0.0, 0.0, ORIENT_TOL)),
                      POS_INC, ORIENT_INC);

  ROS_INFO_STREAM("Initializing fuzzy position/orientation point");
  CartTrajectoryPt fuzzy_both(TolerancedFrame( PositionTolerance(0.0, 0.0, 0.0, POS_TOL),
                                       OrientationTolerance(0.0, 0.0, 0.0, ORIENT_TOL)),
                      POS_INC, ORIENT_INC);


  EigenSTL::vector_Affine3d solutions;
  std::vector<std::vector<double> >joint_solutions;

  CartesianRobot robot;
  fuzzy_pos.getCartesianPoses(robot, solutions);
  EXPECT_EQ(solutions.size(), NUM_SAMPLED_POS);
//  fuzzy_pos.getJointPoses(robot,joint_solutions);
//  EXPECT_EQ(joint_solutions.size(), NUM_SAMPLED_POS);

  fuzzy_orient.getCartesianPoses(robot, solutions);
  EXPECT_EQ(solutions.size(), NUM_SAMPLED_ORIENT);

  fuzzy_both.getCartesianPoses(robot, solutions);
  EXPECT_EQ(solutions.size(), NUM_SAMPLED_BOTH);

}

