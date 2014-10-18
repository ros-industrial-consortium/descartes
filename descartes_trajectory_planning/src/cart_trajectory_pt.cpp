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

#define NOT_IMPLEMENTED_ERR logError("%s not implemented", __PRETTY_FUNCTION__)


namespace descartes
{

CartTrajectoryPt::CartTrajectoryPt():
    tool_base_(Eigen::Affine3d::Identity()),
    tool_pt_(Eigen::Affine3d::Identity()),
    wobj_base_(Eigen::Affine3d::Identity()),
    wobj_pt_(Eigen::Affine3d::Identity())
{}

bool CartTrajectoryPt::getClosestCartPose(const std::vector<double> &seed_state,
                                          const RobotModel &model, Eigen::Affine3d &pose) const
{
  NOT_IMPLEMENTED_ERR;
  return false;
}

bool CartTrajectoryPt::getNominalCartPose(const std::vector<double> &seed_state,
                                          const RobotModel &model, Eigen::Affine3d &pose) const
{
  NOT_IMPLEMENTED_ERR;
  return false;
}

void CartTrajectoryPt::getCartesianPoses(const RobotModel &model, EigenSTL::vector_Affine3d &poses) const
{
  NOT_IMPLEMENTED_ERR;
}

bool CartTrajectoryPt::getClosestJointPose(const std::vector<double> &seed_state,
                                           const RobotModel &model,
                                           std::vector<double> &joint_pose) const
{
  NOT_IMPLEMENTED_ERR;
  return false;
}

bool CartTrajectoryPt::getNominalJointPose(const std::vector<double> &seed_state,
                                           const RobotModel &model,
                                           std::vector<double> &joint_pose) const
{
  NOT_IMPLEMENTED_ERR;
  return false;
}

void CartTrajectoryPt::getJointPoses(const RobotModel &model,
                                     std::vector<std::vector<double> > &joint_poses) const
{
  NOT_IMPLEMENTED_ERR;
}

bool CartTrajectoryPt::isValid(const RobotModel &model) const
{
  NOT_IMPLEMENTED_ERR;
  return false;
}

bool CartTrajectoryPt::setDiscretization(const std::vector<double> &discretization)
{
  NOT_IMPLEMENTED_ERR;
  return false;
}

} /* namespace descartes */
