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

#include "descartes_moveit/utils.h"
#include "descartes_moveit/moveit_state_adapter.h"
#include "descartes_core/pretty_print.hpp"
#include "descartes_moveit/seed_search.h"

#include <eigen_conversions/eigen_msg.h>
#include <random_numbers/random_numbers.h>
#include <ros/assert.h>
#include <sstream>

const static int SAMPLE_ITERATIONS = 10;

namespace
{
bool getJointVelocityLimits(const moveit::core::RobotState& state, const std::string& group_name,
                            std::vector<double>& output)
{
  std::vector<double> result;

  auto models = state.getJointModelGroup(group_name)->getActiveJointModels();
  for (const moveit::core::JointModel* model : models)
  {
    const auto& bounds = model->getVariableBounds();
    // Check to see if there is a single bounds constraint (more might indicate
    // not revolute joint)
    if (model->getType() != moveit::core::JointModel::REVOLUTE &&
        model->getType() != moveit::core::JointModel::PRISMATIC)
    {
      ROS_ERROR_STREAM(__FUNCTION__ << " Unexpected joint type. Currently works only"
                                       " with single axis prismatic or revolute joints.");
      return false;
    }
    else
    {
      result.push_back(bounds[0].max_velocity_);
    }
  }

  output = result;
  return true;
}

}  // end anon namespace

namespace descartes_moveit
{
MoveitStateAdapter::MoveitStateAdapter() : world_to_root_(Eigen::Isometry3d::Identity())
{
}

bool MoveitStateAdapter::initialize(const std::string& robot_description, const std::string& group_name,
                                    const std::string& world_frame, const std::string& tcp_frame)

{
  // Initialize MoveIt state objects
  planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor(robot_description));
  return initialize(psm, group_name, world_frame, tcp_frame);
}

bool MoveitStateAdapter::initialize(planning_scene_monitor::PlanningSceneMonitorPtr& psm, const std::string &group_name,
                                    const std::string &world_frame, const std::string &tcp_frame)
{
  planning_scene_monitor_ = psm;
  planning_scene_monitor_->startSceneMonitor();

  robot_state_.reset(new moveit::core::RobotState(planning_scene_monitor_->getRobotModel()));
  robot_state_->setToDefaultValues();
  getCurrentRobotState();
  joint_group_ = planning_scene_monitor_->getRobotModel()->getJointModelGroup(group_name);

  // Assign robot parameters
  group_name_ = group_name;
  tool_frame_ = tcp_frame;
  world_frame_ = world_frame;

  // Validate our model inputs w/ URDF
  if (!joint_group_)
  {
    CONSOLE_BRIDGE_logError("%s: Joint group '%s' does not exist in robot model", __FUNCTION__, group_name_.c_str());
    std::stringstream msg;
    msg << "Possible group names: " << robot_state_->getRobotModel()->getJointModelGroupNames();
    CONSOLE_BRIDGE_logError(msg.str().c_str());
    return false;
  }

  const auto& link_names = joint_group_->getLinkModelNames();
  if (tool_frame_ != link_names.back())
  {
    CONSOLE_BRIDGE_logError("%s: Tool frame '%s' does not match group tool frame '%s', functionality"
             "will be implemented in the future",
             __FUNCTION__, tool_frame_.c_str(), link_names.back().c_str());
    return false;
  }

  if (!::getJointVelocityLimits(*robot_state_, group_name, velocity_limits_))
  {
    CONSOLE_BRIDGE_logWarn("%s: Could not determine velocity limits of RobotModel from MoveIt", __FUNCTION__);
  }

  if (seed_states_.empty())
  {
    seed_states_ = seed::findRandomSeeds(*robot_state_, group_name_, SAMPLE_ITERATIONS);
    CONSOLE_BRIDGE_logDebug("Generated %lu random seeds", static_cast<unsigned long>(seed_states_.size()));
  }

  auto model_frame = robot_state_->getRobotModel()->getModelFrame();
  if (world_frame_ != model_frame)
  {
    CONSOLE_BRIDGE_logInform("%s: World frame '%s' does not match model root frame '%s', all poses will be"
              " transformed to world frame '%s'",
              __FUNCTION__, world_frame_.c_str(), model_frame.c_str(), world_frame_.c_str());

    Eigen::Isometry3d root_to_world = Eigen::Isometry3d(robot_state_->getFrameTransform(world_frame_));
    world_to_root_ = descartes_core::Frame(root_to_world.inverse());
  }

  // Start the psm
  return true;
}

bool MoveitStateAdapter::getIK(const Eigen::Isometry3d& pose, const std::vector<double>& seed_state,
                               std::vector<double>& joint_pose) const
{
  robot_state_->setJointGroupPositions(group_name_, seed_state);
  return getIK(pose, joint_pose);
}

bool MoveitStateAdapter::getIK(const Eigen::Isometry3d& pose, std::vector<double>& joint_pose) const
{
  bool rtn = false;

  // transform to group base
  Eigen::Isometry3d tool_pose = Eigen::Isometry3d(world_to_root_.frame * pose);

  if (robot_state_->setFromIK(joint_group_, tool_pose, tool_frame_))
  {
    robot_state_->copyJointGroupPositions(group_name_, joint_pose);
    if (!isValid(joint_pose))
    {
      CONSOLE_BRIDGE_logDebug("MoveitStateAdapter.getIK: Robot joint pose is invalid");
    }
    else
    {
      rtn = true;
    }
  }
  else
  {
    rtn = false;
  }

  return rtn;
}

bool MoveitStateAdapter::getAllIK(const Eigen::Isometry3d& pose, std::vector<std::vector<double> >& joint_poses) const
{
  // The minimum difference between solutions should be greater than the search discretization
  // used by the IK solver.  This value is multiplied by 4 to remove any chance that a solution
  // in the middle of a discretization step could be double counted.  In reality, we'd like solutions
  // to be further apart than this.
  double epsilon = 4 * joint_group_->getSolverInstance()->getSearchDiscretization();
  CONSOLE_BRIDGE_logDebug("MoveitStateAdapter.getAllIK: Utilizing an min. difference of %f between IK solutions", epsilon);
  joint_poses.clear();
  for (size_t sample_iter = 0; sample_iter < seed_states_.size(); ++sample_iter)
  {
    std::vector<double> joint_pose;
    if (getIK(pose, joint_pose))
    {
      if (joint_poses.empty())
      {
        std::stringstream msg;
        msg << "MoveitStateAdapter.getAllIK: " << "Found *first* solution on " << sample_iter
            << " iteration, joint: " << joint_pose;
        CONSOLE_BRIDGE_logDebug(msg.str().c_str());
        joint_poses.push_back(joint_pose);
      }
      else
      {
        std::stringstream msg;
        msg << "MoveitStateAdapter.getAllIK: " << "Found *potential* solution on " << sample_iter
            << " iteration, joint: " << joint_pose;
        CONSOLE_BRIDGE_logDebug(msg.str().c_str());

        std::vector<std::vector<double> >::iterator joint_pose_it;
        bool match_found = false;
        for (joint_pose_it = joint_poses.begin(); joint_pose_it != joint_poses.end(); ++joint_pose_it)
        {
          if (descartes_core::utils::equal(joint_pose, (*joint_pose_it), epsilon))
          {
            CONSOLE_BRIDGE_logDebug("MoveitStateAdapter.getAllIK: Found matching, potential solution is not new");
            match_found = true;
            break;
          }
        }
        if (!match_found)
        {
          std::stringstream msg;
          msg << "MoveitStateAdapter.getAllIK: " << "Found *new* solution on " << sample_iter
              << " iteration, joint: " << joint_pose;
          CONSOLE_BRIDGE_logDebug(msg.str().c_str());
          joint_poses.push_back(joint_pose);
        }
      }
    }
    robot_state_->setJointGroupPositions(group_name_, seed_states_[sample_iter]);
  }

  CONSOLE_BRIDGE_logDebug("MoveitStateAdapter.getAllIK: Found %lu joint solutions out of %lu iterations", static_cast<unsigned long>(joint_poses.size()),
                          static_cast<unsigned long>(seed_states_.size()));

  if (joint_poses.empty())
  {
    CONSOLE_BRIDGE_logError("MoveitStateAdapter.getAllIK: Found 0 joint solutions out of %lu iterations", static_cast<unsigned long>(seed_states_.size()));
    return false;
  }
  else
  {
    CONSOLE_BRIDGE_logInform("MoveitStateAdapter.getAllIK: Found %lu joint solutions out of %lu iterations", static_cast<unsigned long>(joint_poses.size()),
                             static_cast<unsigned long>(seed_states_.size()));
    return true;
  }
}

bool MoveitStateAdapter::isInCollision(const std::vector<double>& joint_pose) const
{
  bool in_collision = false;
  if (check_collisions_)
  {
    std::unique_ptr<planning_scene_monitor::LockedPlanningSceneRO> ls(new planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_));

    // Check if we successfully got a locked planning scene
    if (!(*ls))
    {
      CONSOLE_BRIDGE_logError("MoveitStateAdapter.isInCollision: Failed to secure a locked planning scene. Has the planing scene monitor been correctly initialized?");
      return false;
    }

    // Seed with current state in case their are joints outside of the planning group
    robot_state::RobotState robot_state_copy = (*ls)->getCurrentState();
    robot_state_copy.setJointGroupPositions(group_name_, joint_pose);

    // If the state is colliding return false
    in_collision = (*ls)->isStateColliding(robot_state_copy);
  }

  return in_collision;
}

bool MoveitStateAdapter::isInLimits(const std::vector<double> &joint_pose) const
{
  return joint_group_->satisfiesPositionBounds(joint_pose.data());
}

bool MoveitStateAdapter::getFK(const std::vector<double>& joint_pose, Eigen::Isometry3d& pose) const
{
  bool rtn = false;
  getCurrentRobotState();
  robot_state_->setJointGroupPositions(group_name_, joint_pose);
  if (isValid(joint_pose))
  {
    if (robot_state_->knowsFrameTransform(tool_frame_))
    {
      pose = Eigen::Isometry3d(world_to_root_.frame * robot_state_->getFrameTransform(tool_frame_));
      //pose.
      rtn = true;
    }
    else
    {
      CONSOLE_BRIDGE_logError("MoveitStateAdapter.getFK: Robot state does not recognize tool frame: %s", tool_frame_.c_str());
      rtn = false;
    }
  }
  else
  {
    CONSOLE_BRIDGE_logError("MoveitStateAdapter.getFK: Invalid joint pose passed to get forward kinematics");
    rtn = false;
  }

  return rtn;
}

bool MoveitStateAdapter::isValid(const std::vector<double>& joint_pose) const
{
  // Logical check on input sizes
  if (joint_group_->getActiveJointModels().size() != joint_pose.size())
  {
    CONSOLE_BRIDGE_logError("Size of joint pose: %lu doesn't match robot state variable size: %lu",
             static_cast<unsigned long>(joint_pose.size()),
             static_cast<unsigned long>(joint_group_->getActiveJointModels().size()));
    return false;
  }

  // Satisfies joint positional bounds?
  if (!isInLimits(joint_pose))
  {
    CONSOLE_BRIDGE_logDebug("MoveitStateAdapter.isValid: Joint pose does not satisfy positional bounds");
    return false;
  }

  // Is in collision (if collision is active)
  if (isInCollision(joint_pose))
  {
    CONSOLE_BRIDGE_logDebug("MoveitStateAdapter.isValid: Joint pose is in collision");
    return false;
  }

  return true;
}

bool MoveitStateAdapter::isValid(const Eigen::Isometry3d& pose) const
{
  // TODO: Could check robot extents first as a quick check
  std::vector<double> dummy;
  return getIK(pose, dummy);
}

int MoveitStateAdapter::getDOF() const
{
  return joint_group_->getVariableCount();
}

bool MoveitStateAdapter::isValidMove(const double* from_joint_pose,
                                     const double* to_joint_pose, double dt) const
{

  for (int i = 0; i < getDOF(); ++i)
  {
    double dtheta = std::abs(from_joint_pose[i] - to_joint_pose[i]);
    double max_dtheta = dt * velocity_limits_[i];
    if (dtheta > max_dtheta)
      return false;
  }

  return true;
}

std::vector<double> MoveitStateAdapter::getJointVelocityLimits() const
{
  return velocity_limits_;
}

void MoveitStateAdapter::setState(const moveit::core::RobotState& state)
{
  if(static_cast<bool>(robot_state_)){
    CONSOLE_BRIDGE_logDebug("'robot_state_' member pointer is null. Have you called "
                                                  "initialize()?");
  }
  *robot_state_ = state;
  robot_state_->update();
  planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_)->setCurrentState(state);
}

bool MoveitStateAdapter::getCurrentRobotState() const
{
  planning_scene_monitor_->updateFrameTransforms();
  std::unique_ptr<planning_scene_monitor::LockedPlanningSceneRO> ls(new planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_));

  if (!(*ls))
  {
    CONSOLE_BRIDGE_logWarn("MoveitStateAdapter.getCurrentRobotState: Failed to get locked planning scene.");
    return false;
  }
  else
  {
    *robot_state_ = (*ls)->getCurrentState();
    robot_state_->update();
    return true;
  }
}

}  // descartes_moveit
