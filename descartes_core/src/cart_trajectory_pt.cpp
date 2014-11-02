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
#include "descartes_core/cart_trajectory_pt.h"

#define NOT_IMPLEMENTED_ERR(ret) logError("%s not implemented", __PRETTY_FUNCTION__); return ret;


namespace descartes_core
{

CartTrajectoryPt::CartTrajectoryPt():
    point_data_()
{}

CartTrajectoryPt::CartTrajectoryPt(const Frame &wobj_base, const TolerancedFrame &wobj_pt,
                                   const Frame &tool_base, const TolerancedFrame &tool_pt):
  point_data_(wobj_base, wobj_pt, tool_base, tool_pt)
{}

CartTrajectoryPt::CartTrajectoryPt(const TolerancedFrame &wobj_pt):
  point_data_(Frame(Eigen::Affine3d::Identity()), wobj_pt, Frame(Eigen::Affine3d::Identity()), Frame(Eigen::Affine3d::Identity()))
{}

CartTrajectoryPt::CartTrajectoryPt(const Frame &wobj_pt):
    point_data_(Frame(Eigen::Affine3d::Identity()), wobj_pt, Frame(Eigen::Affine3d::Identity()), Frame(Eigen::Affine3d::Identity()))
{}

bool CartTrajectoryPt::getClosestCartPose(const std::vector<double> &seed_state,
                                          const RobotModel &model, Eigen::Affine3d &pose) const
{
  NOT_IMPLEMENTED_ERR(false);
}

bool CartTrajectoryPt::getNominalCartPose(const std::vector<double> &seed_state,
                                          const RobotModel &model, Eigen::Affine3d &pose) const
{
  /* Simply return wobj_pt expressed in world */
  pose = point_data_.wobj_base.frame * point_data_.wobj_pt.frame;
  return true;  //TODO can this ever return false?
}

void CartTrajectoryPt::getCartesianPoses(const RobotModel &model, EigenSTL::vector_Affine3d &poses) const
{
  poses.clear();
}

bool CartTrajectoryPt::getClosestJointPose(const std::vector<double> &seed_state,
                                           const RobotModel &model,
                                           std::vector<double> &joint_pose) const
{
  NOT_IMPLEMENTED_ERR(false);
}

bool CartTrajectoryPt::getNominalJointPose(const std::vector<double> &seed_state,
                                           const RobotModel &model,
                                           std::vector<double> &joint_pose) const
{
  Eigen::Affine3d robot_pose = point_data_.wobj_base.frame * point_data_.wobj_pt.frame *
      point_data_.tool_pt.frame_inv * point_data_.tool_base.frame_inv;
  return model.getIK(robot_pose, seed_state, joint_pose);
}

void CartTrajectoryPt::getJointPoses(const RobotModel &model,
                                     std::vector<std::vector<double> > &joint_poses) const
{
  bool rtn = false;
  Eigen::Affine3d robot_pose = point_data_.wobj_base.frame * point_data_.wobj_pt.frame *
      point_data_.tool_pt.frame_inv * point_data_.tool_base.frame_inv;
  if(model.getAllIK(robot_pose, joint_poses))
  {
    rtn = true;
  }
  else
  {
    logWarn("Call to get joint poses returned empty set");
  }
}

bool CartTrajectoryPt::isValid(const RobotModel &model) const
{
  Eigen::Affine3d robot_pose = point_data_.wobj_base.frame * point_data_.wobj_pt.frame *
      point_data_.tool_pt.frame_inv * point_data_.tool_base.frame_inv;
  return model.isValid(robot_pose);
}

const void* CartTrajectoryPt::getPointData() const
{
  return (const void*) &point_data_;
}

bool CartTrajectoryPt::setSampler(const TrajectoryPtSamplerPtr &sampler)
{
  sampler_ = sampler;
  if (!sampler_->init(*this))
  {
    sampler_.reset();
    return false;
  }
  return true;
}

bool CartTrajectoryPt::sample(size_t n)
{
  if (!sampler_)
  {
    logWarn("No sampler associated with point; Cannot sample.");
    return false;
  }

//  const std::vector<std::vector<double> > solutions* = static_cast<const std::vector<std::vector<double> >* >(sampler_->sample());
//  if (!sampler_->sample())
  return true;
}

} /* namespace descartes_core */
