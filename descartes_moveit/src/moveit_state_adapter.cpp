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

#include <console_bridge/console.h>
#include "descartes_moveit/moveit_state_adapter.h"
#include "eigen_conversions/eigen_msg.h"
#include "random_numbers/random_numbers.h"
#include "descartes_core/pretty_print.hpp"
#include <sstream>


#define NOT_IMPLEMENTED_ERR logError("%s not implemented", __PRETTY_FUNCTION__)

namespace descartes_moveit
{

MoveitStateAdapter::MoveitStateAdapter(const moveit::core::RobotState & robot_state, const std::string & group_name,
                                     const std::string & tool_frame, const std::string & wobj_frame,
                                       const size_t sample_iterations) :
    robot_state_(new moveit::core::RobotState(robot_state)), group_name_(group_name),
  tool_base_(tool_frame), wobj_base_(wobj_frame), sample_iterations_(sample_iterations)
{

  moveit::core::RobotModelConstPtr robot_model_ = robot_state_->getRobotModel();

  if (robot_model_->getJointModelGroup(group_name_))
  {
    std::vector<std::string> joint_names = robot_model_->getLinkModelNames();
    if (tool_base_ != joint_names.back())
    {
      logError("Tool: %s does not match group tool: %s, functionality will be implemented in the future",
               tool_base_.c_str(), joint_names.back().c_str());
    }
    if (wobj_base_ != joint_names.front())
    {
      logError("Work object: %s does not match group base: %s, functionality will be implemented in the future",
               wobj_base_.c_str(), joint_names.front().c_str());
    }
  }
  else
  {
    logError("Joint group: %s does not exist in robot model", group_name_.c_str());
    std::stringstream msg;
    msg << "Possible group names: " << robot_state_->getRobotModel()->getJointModelGroupNames();
    logError(msg.str().c_str());
  }
  return;
}

bool MoveitStateAdapter::getIK(const Eigen::Affine3d &pose, const std::vector<double> &seed_state,
                              std::vector<double> &joint_pose) const
{
  robot_state_->setJointGroupPositions(group_name_, seed_state);
  return getIK(pose, joint_pose);
}

bool MoveitStateAdapter::getIK(const Eigen::Affine3d &pose, std::vector<double> &joint_pose) const
{
  bool rtn = false;
  if (robot_state_->setFromIK(robot_state_->getJointModelGroup(group_name_), pose, tool_base_, 10, 10.0))
  {
    robot_state_->copyJointGroupPositions(group_name_, joint_pose);
    rtn = true;
  }
  else
  {
    logError("Could not set Cartesian pose.");
    rtn = false;
  }
  return rtn;
}

bool MoveitStateAdapter::getAllIK(const Eigen::Affine3d &pose, std::vector<std::vector<double> > &joint_poses) const
{
  //The minimum difference between solutions should be greater than the search discretization
  //used by the IK solver.  This value is multiplied by 2 to remove any chance that a solution
  //in the middle of a discretization step could be double counted.  In reality, we'd like solutions
  //to be further apart than this.
  double epsilon = 2 * robot_state_->getRobotModel()->getJointModelGroup(group_name_)->getSolverInstance()->
      getSearchDiscretization();
  logDebug("Utilizing an min. difference of %f between IK solutions", epsilon);
  joint_poses.clear();
  for (size_t sample_iter = 0; sample_iter < sample_iterations_; ++sample_iter)
  {
    robot_state_->setToRandomPositions();
    std::vector<double> joint_seed;
    robot_state_->copyJointGroupPositions(group_name_, joint_seed);

    std::stringstream msg;
    msg << "Using random seed position " << sample_iter << " iteration, seed: "  << joint_seed;
    logDebug(msg.str().c_str());

    std::vector<double> joint_pose;
    if (getIK(pose, joint_pose))
    {
      if( joint_poses.empty())
      {
        std::stringstream msg;
        msg << "Found *first* solution on " << sample_iter << " iteration, joint: " << joint_pose;
        logDebug(msg.str().c_str());
        joint_poses.push_back(joint_pose);
      }
      else
      {
        std::stringstream msg;
        msg << "Found *potential* solution on " << sample_iter << " iteration, joint: " << joint_pose;
        logDebug(msg.str().c_str());

        std::vector<std::vector<double> >::iterator joint_pose_it;
        for(joint_pose_it = joint_poses.begin(); joint_pose_it != joint_poses.end(); ++joint_pose_it)
        {
          bool new_joint_pose = false;
          for(size_t joint_index = 0; joint_index < (*joint_pose_it).size(); ++joint_index)
          {
            if(fabs(joint_pose[joint_index]-(*joint_pose_it)[joint_index]) > epsilon)
            {
              new_joint_pose = true;
              break;
            }
          }
          if (new_joint_pose)
          {
            std::stringstream msg;
            msg << "Found *new* solution on " << sample_iter << " iteration, joint: " << *joint_pose_it;
            logDebug(msg.str().c_str());
            joint_poses.push_back(joint_pose);
            break;
          }
        }
      }
    }
  }
  logDebug("Found %d joint solutions out of %d iterations", joint_poses.size(), sample_iterations_);
  if (joint_poses.empty())
  {
    return false;
  }
  else
  {
    return true;
  }
}

bool MoveitStateAdapter::getFK(const std::vector<double> &joint_pose, Eigen::Affine3d &pose) const
{
  bool rtn = false;
  robot_state_->setJointGroupPositions(group_name_, joint_pose);
  if ( isValid(joint_pose) )
  {
    if (robot_state_->knowsFrameTransform(tool_base_))
    {
      pose = robot_state_->getFrameTransform(tool_base_);
      rtn = true;
    }
    else
    {
      logError("Robot state does not recognize tool frame: %s", tool_base_.c_str());
      rtn = false;
    }
  }
  else
  {
    logError("Invalid joint pose passed to get forward kinematics");
    rtn = false;
  }
  std::stringstream msg;
  msg << "Returning the pose " << std::endl << pose.matrix() << std::endl
      << "For joint pose: " << joint_pose;
  logInform(msg.str().c_str());
  return rtn;
}

bool MoveitStateAdapter::isValid(const std::vector<double> &joint_pose) const
{
  bool rtn = false;

  if (robot_state_->getJointModelGroup(group_name_)->getActiveJointModels().size() ==
      joint_pose.size())
  {
    robot_state_->setJointGroupPositions(group_name_, joint_pose);
    //TODO: At some point velocities and accelerations should be set for the group as
    //well.
    robot_state_->setVariableVelocities(std::vector<double>(joint_pose.size(), 0.));
    robot_state_->setVariableAccelerations(std::vector<double>(joint_pose.size(), 0.));
    if (robot_state_->satisfiesBounds())
    {
      rtn = true;
    }
    else
    {
      std::stringstream msg;
      msg << "Joint pose: " << joint_pose << ", outside joint boundaries";
      logDebug(msg.str().c_str());
    }
  }
  else
  {
    logError("Size of joint pose: %d doesn't match robot state variable size: %d",
             joint_pose.size(),
             robot_state_->getJointModelGroup(group_name_)->getActiveJointModels().size());
    rtn = false;
  }
  return rtn;
}

bool MoveitStateAdapter::isValid(const Eigen::Affine3d &pose) const
{
  //TODO: Could check robot extents first as a quick check
  std::vector<double> dummy;
  return getIK(pose, dummy);
}

} //descartes_moveit

