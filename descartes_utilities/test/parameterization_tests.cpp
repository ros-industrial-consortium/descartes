/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016, Southwest Research Institute
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
/*
 *  parameterization_tests.cpp
 *
 *  Created on: March 28, 2016
 *  Author: Jonathan Meyer
 */

#include <gtest/gtest.h>

#include "descartes_utilities/parameterization.h"

TEST(toCubicSplines, maintainsPositions)
{
  std::vector<trajectory_msgs::JointTrajectoryPoint> traj;

  const auto dof = 6; // traj width (num joints)
  const auto n = 10; // traj length

  // Build trajectory
  for (auto i = 0; i < n; ++i)
  {
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.time_from_start = ros::Duration(i);
    pt.positions.resize(dof, i);
    traj.push_back(pt);
  }

  // Compute splines
  std::vector<descartes_utilities::SplineInterpolator> splines;
  ASSERT_TRUE(toCubicSplines(traj, splines));
  ASSERT_TRUE(splines.size() == dof);

  // Now we test the control points for each joint and position
  for (auto& pt : traj)
  {
    auto tm = pt.time_from_start.toSec();
    for (auto i = 0; i < dof; ++i)
    {
      auto interpolated_position = splines[i].position(tm);
      auto input_position = pt.positions[i];
      EXPECT_DOUBLE_EQ(interpolated_position, input_position);  
    }
  }
}
