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
 * cart_trajectory_pt.cpp
 *
 *  Created on: Oct 3, 2014
 *      Author: Dan Solomon
 */

#include <console_bridge/console.h>
#include "descartes_trajectory_planning/cart_trajectory_pt.h"

#define NOT_IMPLEMENTED_ERR(ret) logError("%s not implemented", __PRETTY_FUNCTION__); return ret;


namespace descartes
{

CartTrajectoryPt::CartTrajectoryPt():
    tool_base_(Eigen::Affine3d::Identity()),
    tool_pt_(Eigen::Affine3d::Identity()),
    wobj_base_(Eigen::Affine3d::Identity()),
    wobj_pt_(Eigen::Affine3d::Identity())
{}

bool CartTrajectoryPt::getClosestCartPose(Eigen::Affine3d &pose, const moveit::core::RobotState &seed_state) const
{
  NOT_IMPLEMENTED_ERR(false)
}

bool CartTrajectoryPt::getNominalCartPose(Eigen::Affine3d &pose, const moveit::core::RobotState &seed_state) const
{
  /* Simply return wobj_pt expressed in world */
  pose = wobj_base_.frame * wobj_pt_.frame;
  return true;  //TODO can this ever return false?
}

bool CartTrajectoryPt::getClosestJointPose(std::vector<double> &joint_pose, const moveit::core::RobotState &seed_state) const
{
  NOT_IMPLEMENTED_ERR(false)
}

bool CartTrajectoryPt::getNominalJointPose(std::vector<double> &joint_pose, const moveit::core::RobotState &seed_state) const
{
  moveit::core::RobotState state(seed_state);
  Eigen::Affine3d robot_pose = wobj_base_.frame * wobj_pt_.frame * tool_pt_.frame_inv * tool_base_.frame_inv;
  std::string group_name("manipulator");        //TODO get from somewhere
  if (!state.setFromIK(state.getJointModelGroup(group_name), robot_pose))
  {
    logError("Could not set Cartesian pose.");
    return false;
  }
  joint_pose.resize(state.getVariableCount());
  for (size_t ii=0; ii<state.getVariableCount(); ++ii)
  {
    joint_pose[ii] = state.getVariablePosition(ii);
  }
  return true;
}

bool CartTrajectoryPt::isValid(const moveit::core::RobotState &state) const
{
  NOT_IMPLEMENTED_ERR(false)
}

} /* namespace descartes */
