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
 *  parameterization.cpp
 *
 *  Created on: March 28, 2016
 *  Author: Jonathan Meyer
 */

#include "descartes_utilities/parameterization.h"
#include <console_bridge/console.h>

bool descartes_utilities::toCubicSplines(const std::vector<trajectory_msgs::JointTrajectoryPoint>& traj,
                                         std::vector<SplineInterpolator>& splines)
{
  if (traj.size() < 2)
  {
    logError("%s: Can not parameterize trajectory with fewer than 2 points", __FUNCTION__);
    return false;
  }

  // determine size of trajectory points
  const auto n = traj.size();
  const auto dof = traj.front().positions.size();
  
  // build data structures for time and position at each point
  std::vector<std::vector<double>> joint_values (dof);
  std::vector<double> time_values (n);
  for (auto& joint_vec : joint_values)
  {
    joint_vec.resize(n);
  }

  auto last_time_seen = traj.front().time_from_start.toSec();

  // Fill out data structures
  for (std::size_t i = 0; i < traj.size(); ++i)
  {
    // check time to ensure consistency
    const auto tm = traj[i].time_from_start.toSec();
    if (tm < last_time_seen)
    {
      logError("%s: 'time_from_start' fields must be sorted in increasing order", __FUNCTION__);
      return false;
    }

    // copy time
    time_values[i] = tm;
    // update last time seen
    last_time_seen = tm;
    
    // check to make sure nothing funky about the size of the positions
    if (traj[i].positions.size() != dof)
    {
      logError("%s: 'Positions' field of trajectory point at index %u does not have the same size as first",
        __FUNCTION__, static_cast<unsigned>(i));
      return false;
    }

    // copy joints
    for (std::size_t j = 0; j < dof; ++j)
    {
      joint_values[j][i] = traj[i].positions[j];
    }
  }

  // fit splines
  for (const auto& joint_vec : joint_values)
  {
    splines.emplace_back(time_values, joint_vec, 0.0, 0.0);
  }

  return true;
}

bool descartes_utilities::setDerivatesFromSplines(std::vector<trajectory_msgs::JointTrajectoryPoint>& traj)
{
  std::vector<SplineInterpolator> splines;
  if (!toCubicSplines(traj, splines))
  {
    logError("%s: Unable to compute splines for input trajectory", __FUNCTION__);
    return false;
  }

  const auto dof = splines.size();

  // Sample each parameter at the nominal time point
  for (auto& pt : traj)
  {
    auto tm = pt.time_from_start.toSec();

    // ensure there is room
    if (pt.velocities.size() != dof) pt.velocities.resize(dof);
    if (pt.accelerations.size() != dof) pt.accelerations.resize(dof);

    // sample values
    for (std::size_t j = 0; j < dof; ++j)
    {
      pt.velocities[j] = splines[j].velocity(tm);
      pt.accelerations[j] = splines[j].acceleration(tm);
    }
  }

  return true;
}
