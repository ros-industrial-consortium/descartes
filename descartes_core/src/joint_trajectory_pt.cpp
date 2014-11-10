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
#include "descartes_core/joint_trajectory_pt.h"


#define NOT_IMPLEMENTED_ERR(ret) logError("%s not implemented", __PRETTY_FUNCTION__); return ret;


namespace descartes_core
{

JointTrajectoryPt::JointTrajectoryPt():
    point_data_()
{}

JointTrajectoryPt::JointTrajectoryPt(const std::vector<TolerancedJointValue> &joints,
                                     const Frame &tool, const Frame &wobj):
  point_data_(joints, tool, wobj)
{}

JointTrajectoryPt::JointTrajectoryPt(const std::vector<TolerancedJointValue> &joints):
  point_data_(joints, Eigen::Affine3d::Identity(), Eigen::Affine3d::Identity())
{}

JointTrajectoryPt::JointTrajectoryPt(const std::vector<double> &joints):
  point_data_(std::vector<TolerancedJointValue>(), Eigen::Affine3d::Identity(), Eigen::Affine3d::Identity())
{
  for (size_t ii = 0; ii < joints.size(); ++ii)
  {
    point_data_.joint_position.push_back(TolerancedJointValue(joints[ii]));
  }
}


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

const void* JointTrajectoryPt::getPointData() const
{
  return (const void*) &point_data_;
}

bool JointTrajectoryPt::getNominalJointPose(const std::vector<double> &seed_state,
                                            const RobotModel &model,
                                            std::vector<double> &joint_pose) const
{
  joint_pose.resize(point_data_.joint_position.size());
  for (size_t ii=0; ii<point_data_.joint_position.size(); ++ii)
  {
    joint_pose[ii] = point_data_.joint_position[ii].nominal;
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
  std::vector<double> lower(point_data_.joint_position.size());
  std::vector<double> upper(point_data_.joint_position.size());
  for (size_t ii = 0; ii < point_data_.joint_position.size(); ++ii)
  {
    lower[ii] = point_data_.joint_position[ii].tolerance.lower;
    upper[ii] = point_data_.joint_position[ii].tolerance.upper;
  }
return model.isValid(lower) && model.isValid(upper);
}

bool JointTrajectoryPt::setSampler(const TrajectoryPtSamplerPtr &sampler)
{
  sampler_ = sampler;
  if (!sampler_->init(*this))
  {
    sampler_.reset();
    return false;
  }
  return true;
}

bool JointTrajectoryPt::sample(size_t n)
{
  if (!sampler_)
  {
    logWarn("No sampler associated with point; Cannot sample.");
    return false;
  }

  const std::vector<std::vector<double> > *solutions = static_cast<const std::vector<std::vector<double> >* >(sampler_->sample(n, *this));
  return true;
}

} /* namespace descartes_core */
