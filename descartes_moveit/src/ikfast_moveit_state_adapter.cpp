/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016, Jonathan Meyer
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

#include "descartes_moveit/ikfast_moveit_state_adapter.h"
#include "eigen_conversions/eigen_msg.h"

bool descartes_moveit::IkFastMoveitStateAdapter::getAllIK(const Eigen::Affine3d &pose, 
  std::vector<std::vector<double> > &joint_poses) const
{
  joint_poses.clear();
  const auto& solver = joint_group_->getSolverInstance();

  // Transform input pose
  Eigen::Affine3d tool_pose = world_to_root_.frame * pose;

  // convert to geometry_msgs ...
  geometry_msgs::Pose geometry_pose;
  tf::poseEigenToMsg(tool_pose, geometry_pose);
  std::vector<geometry_msgs::Pose> poses = {geometry_pose};

  std::vector<double> dummy_seed (getDOF(), 0.0);
  std::vector<std::vector<double>> joint_results;
  kinematics::KinematicsResult result;
  kinematics::KinematicsQueryOptions options; // defaults are reasonable as of Indigo

  if (!solver->getPositionIK(poses, dummy_seed, joint_results, result, options))
  {
    return false;
  }

  for (auto& sol : joint_results)
  {
    if (isValid(sol)) joint_poses.push_back(std::move(sol));
  }

  return joint_poses.size() > 0;
}
