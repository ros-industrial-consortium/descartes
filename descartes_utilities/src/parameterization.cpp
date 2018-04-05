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
    if (i > 0 && tm <= last_time_seen)
    {
      logError("%s: 'time_from_start' fields must be sorted in increasing order (index %lu = %f, index %lu = %f)", 
        __FUNCTION__, static_cast<unsigned long>(i-1), last_time_seen, static_cast<unsigned long>(i), tm);
      return false;
    }

    // copy time
    time_values[i] = tm;
    // update last time seen
    last_time_seen = tm;
    
    // check to make sure nothing funky about the size of the positions
    if (traj[i].positions.size() != dof)
    {
      logError("%s: 'Positions' field of trajectory point at index %lu does not have the same size as first (%lu vs %lu)",
        __FUNCTION__, static_cast<unsigned long>(i), static_cast<unsigned long>(dof), 
        static_cast<unsigned long>(traj[i].positions.size()));
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

  return setDerivatesFromSplines(splines, traj);
}

bool descartes_utilities::setDerivatesFromSplines(const std::vector<SplineInterpolator>& splines,
                                                  std::vector<trajectory_msgs::JointTrajectoryPoint>& traj)
{
  const auto dof = splines.size();

  // Sample each parameter at the nominal time point
  for (auto& pt : traj)
  {
    auto tm = pt.time_from_start.toSec();

    // sanity check the DOF of the splines with the DOF of the input path
    if (pt.positions.size() != dof)
    {
      logError("%s: Splines vector is of size %lu and input trajectory point (time %f) has 'positions' field of size %lu",
        __FUNCTION__, static_cast<unsigned long>(dof), tm, static_cast<unsigned long>(pt.positions.size()));
      return false;
    }

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

bool descartes_utilities::resampleTrajectory(const std::vector<SplineInterpolator>& splines,
                                             double start_tm, double end_tm, double tm_step,
                                             std::vector<trajectory_msgs::JointTrajectoryPoint>& traj)
{
  if (start_tm >= end_tm)
  {
    logError("%s: 'start_tm' must be greater than or equal to 'end_tm' (%f vs %f)", __FUNCTION__,
      start_tm, end_tm);
    return false;
  }

  const auto dof = splines.size();
  std::vector<trajectory_msgs::JointTrajectoryPoint> new_traj;

  for (double tm = start_tm; tm < end_tm; tm += tm_step)
  {
    auto pt = trajectory_msgs::JointTrajectoryPoint();
    pt.positions.resize(dof);
    pt.velocities.resize(dof);
    pt.accelerations.resize(dof);
    pt.time_from_start = ros::Duration(tm);

    for (auto i = 0; i < dof; i++)
    {
      pt.positions[i] = splines[i].position(tm);
      pt.velocities[i] = splines[i].velocity(tm);
      pt.accelerations[i] = splines[i].acceleration(tm);
    }

    new_traj.push_back(pt);
  }

  traj = new_traj;
  return true;
}
