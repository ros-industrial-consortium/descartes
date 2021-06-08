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
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(robot_description));
  auto model = robot_model_loader_->getModel();

  ROS_INFO_STREAM("initialize (1): group " << group_name << " tcp " << tcp_frame);

  if (!model)
  {
    CONSOLE_BRIDGE_logError("Failed to load robot model from robot description parameter: %s", robot_description.c_str());
    return false;
  }

  return initialize(model, group_name, world_frame, tcp_frame);
}

bool MoveitStateAdapter::initialize(robot_model::RobotModelConstPtr robot_model, const std::string &group_name,
                                    const std::string &world_frame, const std::string &tcp_frame)
{
  robot_model_ptr_ = robot_model;
  robot_state_.reset(new moveit::core::RobotState(robot_model_ptr_));
  robot_state_->setToDefaultValues();
  planning_scene_.reset(new planning_scene::PlanningScene(robot_model));
  joint_group_ = robot_model_ptr_->getJointModelGroup(group_name);

  ROS_INFO_STREAM("initialize (2): group " << group_name << " tcp " << tcp_frame);

  // Assign robot frames
  group_name_ = group_name;
  tool_frame_ = tcp_frame;
  world_frame_ = world_frame;

  // Validate our model inputs w/ URDF
  if (!joint_group_)
  {
    CONSOLE_BRIDGE_logError("%s: Joint group '%s' does not exist in robot model", __FUNCTION__, group_name_.c_str());
    std::stringstream msg;

    msg << "Possible group names: ";
    for (const auto& name : robot_state_->getRobotModel()->getJointModelGroupNames())
      msg << name << ", ";
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
              " transformed to world frame '%s'. Bummerrrr...",
              __FUNCTION__, world_frame_.c_str(), model_frame.c_str(), world_frame_.c_str());

    // FIXME: This was commented out in the first peanut commit after branching from kinetic-devel
    // need to understand why and add a comment here, or undo the change...
    // <<<<<<< HEAD (upstream version of melodic-devel)
    //     Eigen::Isometry3d root_to_world = toIsometry(robot_state_->getFrameTransform(world_frame_));
    //     world_to_root_ = descartes_core::Frame(root_to_world.inverse());
    // =======
    // //    Eigen::Affine3d root_to_world = robot_state_->getFrameTransform(world_frame_);
    // //    world_to_root_ = descartes_core::Frame(root_to_world.inverse());
    // >>>>>>> kinetic-devel (peanut version)
  }

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
    // FIXME: This was changed in the first peanut commit after branching from kinetic-devel
    // need to understand why and add a comment here, or undo the change...
    // <<<<<<< HEAD (upstream melodic-devel)
    //   Eigen::Isometry3d tool_pose = world_to_root_.frame * pose;
    // =======
    // //  Eigen::Affine3d tool_pose = world_to_root_.frame * pose;
    //   Eigen::Affine3d tool_pose = pose;
    // >>>>>>> kinetic-devel (peanut version)
    Eigen::Isometry3d tool_pose = pose;

  if (robot_state_->setFromIK(joint_group_, tool_pose, tool_frame_))
  {
    robot_state_->copyJointGroupPositions(group_name_, joint_pose);
    if (!isValid(joint_pose))
    {
      ROS_DEBUG_STREAM("Robot joint pose is invalid");
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
  CONSOLE_BRIDGE_logDebug("Utilizing an min. difference of %f between IK solutions", epsilon);
  joint_poses.clear();
  for (size_t sample_iter = 0; sample_iter < seed_states_.size(); ++sample_iter)
  {
    robot_state_->setJointGroupPositions(group_name_, seed_states_[sample_iter]);
    std::vector<double> joint_pose;
    if (getIK(pose, joint_pose))
    {
      if (joint_poses.empty())
      {
        std::stringstream msg;
        CONSOLE_BRIDGE_logDebug(msg.str().c_str());
        joint_poses.push_back(joint_pose);
      }
      else
      {
        std::stringstream msg;
        CONSOLE_BRIDGE_logDebug(msg.str().c_str());

        std::vector<std::vector<double> >::iterator joint_pose_it;
        bool match_found = false;
        for (joint_pose_it = joint_poses.begin(); joint_pose_it != joint_poses.end(); ++joint_pose_it)
        {
          if (descartes_core::utils::equal(joint_pose, (*joint_pose_it), epsilon))
          {
            CONSOLE_BRIDGE_logDebug("Found matching, potential solution is not new");
            match_found = true;
            break;
          }
        }
        if (!match_found)
        {
          std::stringstream msg;
          CONSOLE_BRIDGE_logDebug(msg.str().c_str());
          joint_poses.push_back(joint_pose);
        }
      }
    }
  }

  CONSOLE_BRIDGE_logDebug("Found %lu joint solutions out of %lu iterations", static_cast<unsigned long>(joint_poses.size()),
           static_cast<unsigned long>(seed_states_.size()));

  if (joint_poses.empty())
  {
    CONSOLE_BRIDGE_logError("Found 0 joint solutions out of %lu iterations", static_cast<unsigned long>(seed_states_.size()));
    return false;
  }
  else
  {
    CONSOLE_BRIDGE_logInform("Found %lu joint solutions out of %lu iterations", static_cast<unsigned long>(joint_poses.size()),
              static_cast<unsigned long>(seed_states_.size()));
    return true;
  }
}

bool MoveitStateAdapter::isInCollision(const std::vector<double>& joint_pose) const
{
  bool in_collision = false;
  if (check_collisions_)
  {
    moveit::core::RobotState state (robot_model_ptr_);
    state.setToDefaultValues();
    state.setJointGroupPositions(joint_group_, joint_pose);
    in_collision = planning_scene_->isStateColliding(state, group_name_);
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
  robot_state_->setJointGroupPositions(group_name_, joint_pose);
  if (isValid(joint_pose))
  {
    if (robot_state_->knowsFrameTransform(tool_frame_))
    {
      pose = toIsometry(world_to_root_.frame * robot_state_->getFrameTransform(tool_frame_));
      //pose.
      rtn = true;
    }
    else
    {
      CONSOLE_BRIDGE_logError("Robot state does not recognize tool frame: %s", tool_frame_.c_str());
      rtn = false;
    }
  }
  else
  {
    CONSOLE_BRIDGE_logError("Invalid joint pose passed to get forward kinematics");
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

  return isInLimits(joint_pose) && !isInCollision(joint_pose);
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
  ROS_ASSERT_MSG(static_cast<bool>(robot_state_), "'robot_state_' member pointer is null. Have you called "
                                                  "initialize()?");
  *robot_state_ = state;
  planning_scene_->setCurrentState(state);
}

}  // descartes_moveit
