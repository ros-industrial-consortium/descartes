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

bool JointTrajectoryPt::getClosestCartPose(const std::vector<double> &seed_state,
                                           const RobotModel &model, Eigen::Affine3d &pose) const
{
  NOT_IMPLEMENTED_ERR(false)
}

bool JointTrajectoryPt::getNominalCartPose(const std::vector<double> &seed_state,
                                           const RobotModel &model, Eigen::Affine3d &pose) const
{
  NOT_IMPLEMENTED_ERR(false)
}

void JointTrajectoryPt::getCartesianPoses(const RobotModel &model, EigenSTL::vector_Affine3d &poses) const
{
  poses.clear();
}

bool JointTrajectoryPt::getClosestJointPose(const std::vector<double> &seed_state,
                                            const RobotModel &model,
                                            std::vector<double> &joint_pose) const
{
  NOT_IMPLEMENTED_ERR(false);
}

bool JointTrajectoryPt::getNominalJointPose(const std::vector<double> &seed_state,
                                            const RobotModel &model,
                                            std::vector<double> &joint_pose) const
{
  joint_pose.resize(joint_position_.size());
  for (size_t ii=0; ii<joint_position_.size(); ++ii)
  {
    joint_pose[ii] = joint_position_[ii].nominal;
  }
  return true;
}

void JointTrajectoryPt::getJointPoses(const RobotModel &model,
                                      std::vector<std::vector<double> > &joint_poses) const
{
  joint_poses.clear();
}

bool JointTrajectoryPt::isValid(const RobotModel &model) const
{
  std::vector<double> lower(joint_position_.size());
  std::vector<double> upper(joint_position_.size());
  for (size_t ii = 0; ii < joint_position_.size(); ++ii)
  {
    lower[ii] = joint_position_[ii].tolerance.lower;
    upper[ii] = joint_position_[ii].tolerance.upper;
  }
return model.isValid(lower) && model.isValid(upper);
}
} /* namespace descartes */
