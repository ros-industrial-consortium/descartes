/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Dan Solomon
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
 * joint_trajectory_pt.cpp
 *
 *  Created on: Oct 3, 2014
 *      Author: Dan Solomon
 */

#include <console_bridge/console.h>
#include "descartes_trajectory_planning/joint_trajectory_pt.h"

#define NOT_IMPLEMENTED_ERR(ret) logError("%s not implemented", __PRETTY_FUNCTION__); return ret;


namespace descartes
{

JointTrajectoryPt::JointTrajectoryPt():
    tool_(Eigen::Affine3d::Identity()),
    wobj_(Eigen::Affine3d::Identity())
{}

bool JointTrajectoryPt::getClosestCartPose(Eigen::Affine3d &pose, const moveit::core::RobotState &seed_state) const
{
  NOT_IMPLEMENTED_ERR(false)
}

bool JointTrajectoryPt::getNominalCartPose(Eigen::Affine3d &pose, const moveit::core::RobotState &seed_state) const
{
  NOT_IMPLEMENTED_ERR(false)
}

bool JointTrajectoryPt::getClosestJointPose(std::vector<double> &joint_pose, const moveit::core::RobotState &seed_state) const
{
  NOT_IMPLEMENTED_ERR(false)
}

bool JointTrajectoryPt::getNominalJointPose(std::vector<double> &joint_pose, const moveit::core::RobotState &seed_state) const
{
  moveit::core::RobotState state(seed_state);
  std::vector<double> nominal(getNominalJointPose());
  state.setVariablePositions(nominal);
  state.setVariableVelocities(std::vector<double>(nominal.size(), 0.));
  state.setVariableAccelerations(std::vector<double>(nominal.size(), 0.));
  if (state.satisfiesBounds())
  {
    joint_pose = nominal;
    return true;
  }
  return false;
}

std::vector<double> JointTrajectoryPt::getNominalJointPose() const
{
  std::vector<double> nominal(joint_position_.size());
  for (size_t ii=0; ii<joint_position_.size(); ++ii)
  {
    nominal[ii] = joint_position_[ii].nominal;
  }
  return nominal;
}

bool JointTrajectoryPt::isValid(const moveit::core::RobotState &state) const
{
  if (joint_position_.size() > state.getVariableCount())
  {
    logError("Variables in RobotState must be >= joints listed in JointTrajectoryPt.");
    return false;
  }
  if (joint_position_.size() != state.getVariableCount())
  {
    logWarn("Mismatched size between joint point and state.");
  }

  for (int ii=0; ii<joint_position_.size(); ++ii)
  {
    const double &state_joint = state.getVariablePosition(ii);
    if (state_joint > joint_position_[ii].upperBound() || state_joint < joint_position_[ii].lowerBound())
    {
      return false;
    }
  }
  return true;
}

} /* namespace descartes */
