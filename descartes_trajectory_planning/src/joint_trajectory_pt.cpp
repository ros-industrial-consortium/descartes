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

#define NOT_IMPLEMENTED_ERR logError("%s not implemented", __PRETTY_FUNCTION__)


namespace descartes
{

JointTrajectoryPt::JointTrajectoryPt()
{
  // TODO Auto-generated constructor stub

}

JointTrajectoryPt::~JointTrajectoryPt()
{
  // TODO Auto-generated destructor stub
}

bool JointTrajectoryPt::getClosestCartPose(Eigen::Affine3d &pose, const moveit::core::RobotState &seed_state) const
{
  NOT_IMPLEMENTED_ERR;
  return false;
}

bool JointTrajectoryPt::getNominalCartPose(Eigen::Affine3d &pose, const moveit::core::RobotState &seed_state) const
{
  NOT_IMPLEMENTED_ERR;
  return false;
}

void JointTrajectoryPt::getCartesianPoses(EigenSTL::vector_Affine3d &poses, const moveit::core::RobotState &state) const
{
  NOT_IMPLEMENTED_ERR;
}

bool JointTrajectoryPt::getClosestJointPose(std::vector<double> &joint_pose, const moveit::core::RobotState &seed_state) const
{
  NOT_IMPLEMENTED_ERR;
  return false;
}

bool JointTrajectoryPt::getNominalJointPose(std::vector<double> &joint_pose, const moveit::core::RobotState &seed_state) const
{
  NOT_IMPLEMENTED_ERR;
  return false;
}

void JointTrajectoryPt::getJointPoses(std::vector<std::vector<double> > &joint_poses, const moveit::core::RobotState &state) const
{
  NOT_IMPLEMENTED_ERR;
}

bool JointTrajectoryPt::isValid(const moveit::core::RobotState &state) const
{
  if (joint_position_.size() > state.getVariableCount())
  {
    logError("Variables in RobotState must be >= joints listed in JointTrajectoryPt.");
    return false;
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

bool JointTrajectoryPt::setDiscretization(const std::vector<double> &discretization)
{
  if (discretization.size() != 1 || discretization.size() != joint_position_.size())
  {
    logError("discretization must be size 1 or same size as joint count.");
    return false;
  }

  if (discretization.size() == 1)
  {
    discretization_ = std::vector<double>(joint_position_.size(), discretization[0]);
    return true;
  }

  /* Do not copy discretization values until all values are confirmed */
  for (size_t ii=0; ii<discretization.size(); ++ii)
  {
    if (discretization[ii] < 0. || discretization[ii] > joint_position_[ii].range())
    {
      logError("discretization value out of range.");
      return false;
    }
  }

  discretization_ = discretization;

  return true;
}

} /* namespace descartes */
