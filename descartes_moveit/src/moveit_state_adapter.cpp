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

const static int SAMPLE_ITERATIONS = 10;

const static double MAX_ITER_SEARCH_TIME = 0.05;

namespace descartes_moveit
{

MoveitStateAdapter::MoveitStateAdapter()
{

}

MoveitStateAdapter::MoveitStateAdapter(const moveit::core::RobotState & robot_state, const std::string & group_name,
                                     const std::string & tool_frame, const std::string & world_frame,
                                       const size_t sample_iterations) :
    robot_state_(new moveit::core::RobotState(robot_state)), group_name_(group_name),
  tool_frame_(tool_frame), world_frame_(world_frame), sample_iterations_(sample_iterations),
  world_to_root_(Eigen::Affine3d::Identity())
{

  moveit::core::RobotModelConstPtr robot_model_ = robot_state_->getRobotModel();
  const moveit::core::JointModelGroup* joint_model_group_ptr = robot_state_->getJointModelGroup(group_name);
  if (joint_model_group_ptr)
  {
    joint_model_group_ptr->printGroupInfo();

    const std::vector<std::string>& link_names = joint_model_group_ptr->getLinkModelNames();
    if (tool_frame_ != link_names.back())
    {
      logWarn("Tool frame '%s' does not match group tool frame '%s', functionality will be implemented in the future",
               tool_frame_.c_str(), link_names.back().c_str());
    }

    if (world_frame_ != robot_state_->getRobotModel()->getModelFrame())
    {
      logWarn("World frame '%s' does not match model root frame '%s', all poses will be transformed to world frame '%s'",
               world_frame_.c_str(), link_names.front().c_str(),world_frame_.c_str());

      Eigen::Affine3d root_to_world = robot_state_->getFrameTransform(world_frame_);
      world_to_root_ = descartes_core::Frame(root_to_world.inverse());
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

bool MoveitStateAdapter::initialize(const std::string robot_description, const std::string& group_name,
                                    const std::string& world_frame,const std::string& tcp_frame)
{

  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(robot_description));
  robot_model_ptr_ = robot_model_loader_->getModel();
  robot_state_.reset(new moveit::core::RobotState(robot_model_ptr_));
  group_name_ = group_name;
  tool_frame_ = tcp_frame;
  world_frame_ = world_frame;
  sample_iterations_ = SAMPLE_ITERATIONS;

  const moveit::core::JointModelGroup* joint_model_group_ptr = robot_state_->getJointModelGroup(group_name);
  if (joint_model_group_ptr)
  {
    joint_model_group_ptr->printGroupInfo();

    const std::vector<std::string>& link_names = joint_model_group_ptr->getLinkModelNames();
    if (tool_frame_ != link_names.back())
    {
      logWarn("Tool frame '%s' does not match group tool frame '%s', functionality will be implemented in the future",
               tool_frame_.c_str(), link_names.back().c_str());
    }

    if (world_frame_ != robot_state_->getRobotModel()->getModelFrame())
    {
      logWarn("World frame '%s' does not match model root frame '%s', all poses will be transformed to world frame '%s'",
               world_frame_.c_str(), link_names.front().c_str(),world_frame_.c_str());

      Eigen::Affine3d root_to_world = robot_state_->getFrameTransform(world_frame_);
      world_to_root_ = descartes_core::Frame(root_to_world.inverse());
    }

  }
  else
  {
    logError("Joint group: %s does not exist in robot model", group_name_.c_str());
    std::stringstream msg;
    msg << "Possible group names: " << robot_state_->getRobotModel()->getJointModelGroupNames();
    logError(msg.str().c_str());
  }
  return true;
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

  // transform to group base
  Eigen::Affine3d tool_pose = world_to_root_.frame* pose;


  if (robot_state_->setFromIK(robot_state_->getJointModelGroup(group_name_), tool_pose,
                              tool_frame_, 1, MAX_ITER_SEARCH_TIME))
  {
    robot_state_->copyJointGroupPositions(group_name_, joint_pose);
    rtn = true;
  }
  else
  {
    rtn = false;
  }
  return rtn;
}

bool MoveitStateAdapter::getAllIK(const Eigen::Affine3d &pose, std::vector<std::vector<double> > &joint_poses) const
{
  //The minimum difference between solutions should be greater than the search discretization
  //used by the IK solver.  This value is multiplied by 4 to remove any chance that a solution
  //in the middle of a discretization step could be double counted.  In reality, we'd like solutions
  //to be further apart than this.
  double epsilon = 4 * robot_state_->getRobotModel()->getJointModelGroup(group_name_)->getSolverInstance()->
      getSearchDiscretization();
  logDebug("Utilizing an min. difference of %f between IK solutions", epsilon);
  joint_poses.clear();
  for (size_t sample_iter = 0; sample_iter < sample_iterations_; ++sample_iter)
  {
    robot_state_->setToRandomPositions();
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
        bool match_found = false;
        for(joint_pose_it = joint_poses.begin(); joint_pose_it != joint_poses.end(); ++joint_pose_it)
        {
          if( descartes_core::utils::equal(joint_pose, (*joint_pose_it), epsilon) )
          {
            logDebug("Found matching, potential solution is not new");
            match_found = true;
            break;
          }
        }
        if (!match_found)
        {
          std::stringstream msg;
          msg << "Found *new* solution on " << sample_iter << " iteration, joint: " << joint_pose;
          logDebug(msg.str().c_str());
          joint_poses.push_back(joint_pose);
        }
      }
    }
  }
  logDebug("Found %d joint solutions out of %d iterations", joint_poses.size(), sample_iterations_);
  if (joint_poses.empty())
  {
    logError("Found 0 joint solutions out of %d iterations", sample_iterations_);
    return false;
  }
  else
  {
    logInform("Found %d joint solutions out of %d iterations", joint_poses.size(), sample_iterations_);
    return true;
  }
}

bool MoveitStateAdapter::getFK(const std::vector<double> &joint_pose, Eigen::Affine3d &pose) const
{
  bool rtn = false;
  robot_state_->setJointGroupPositions(group_name_, joint_pose);
  if ( isValid(joint_pose) )
  {
    if (robot_state_->knowsFrameTransform(tool_frame_))
    {
      pose = world_to_root_.frame*robot_state_->getFrameTransform(tool_frame_);
      rtn = true;
    }
    else
    {
      logError("Robot state does not recognize tool frame: %s", tool_frame_.c_str());
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
  logDebug(msg.str().c_str());
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

int MoveitStateAdapter::getDOF() const
{
  const moveit::core::JointModelGroup* group;
  group = robot_state_->getJointModelGroup(group_name_);
  return group->getVariableCount();
}

descartes_core::RobotModelPtr MoveitStateAdapter::clone() const
{
  return descartes_core::RobotModelPtr(new MoveitStateAdapter(*robot_state_, group_name_, tool_frame_, world_frame_, sample_iterations_));
}

} //descartes_moveit

