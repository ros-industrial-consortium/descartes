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

const static int SAMPLE_ITERATIONS = 10;

namespace seed_search
{
typedef std::vector<double> JointConfig;
typedef std::vector<JointConfig> JointConfigVec;

class CombinationManager
{
public:
  struct ComboHandle
  {
    ComboHandle(size_t n)
      : combos_(n)
    {}

    uint8_t& operator[](size_t idx)
    {
      return combos_[idx];
    }

    const uint8_t& operator[](size_t idx) const
    {
      return combos_[idx];
    }

    const std::size_t size() const { return combos_.size(); }

    std::vector<uint8_t> combos_;
  };

  CombinationManager(unsigned n_joints, unsigned n_discs)
    : n_joints_(n_joints)
    , n_discs_(n_discs)
  {}

  size_t count() const
  {
    size_t result = 1;
    for (size_t i = 0; i < n_joints_; ++i) result *= n_discs_;
    return result;
  }

  ComboHandle fromNumber(size_t n)
  {
    ComboHandle result (n_joints_);
    for (size_t i = 0; i < n_joints_; i++)
    {
      result[n_joints_-1-i] = static_cast<uint8_t>(n % n_discs_);
      n /= n_discs_;
    }
    return result;
  }

private:
  unsigned n_joints_;
  unsigned n_discs_;
};

bool operator<(const CombinationManager::ComboHandle& lhs, const CombinationManager::ComboHandle& rhs)
{
  for (std::size_t i = 0; i < lhs.size(); ++i)
  {
    if ( lhs[i] != rhs[i] )
    {
      return lhs[i] < rhs[i];
    }
  }
  // equal
  return false;
}

unsigned FIRST_N = 2;

typedef std::vector<unsigned> BijectionVec;

JointConfig toJointConfig(const CombinationManager::ComboHandle& handle, const BijectionVec& bij)
{
  const static double min = -M_PI;
  const static double step = M_PI/2;

  const static unsigned DOF = 6;

  JointConfig result;
  result.resize(DOF);

  std::size_t i;
  for (i = 0; i < handle.size(); i++)
  {
    result[bij[i]] = min + handle[i] * step;
  }

  for (; i < bij.size(); ++i)
  {
    result[bij[i]] = 0.0;
  }

  result[4] = 1.57;

  return result;
}

bool doFK(moveit::core::RobotState& state,
          const moveit::core::JointModelGroup* group,
          const std::string& tool,
          const JointConfig& joint_pose,
          Eigen::Affine3d& result)
{
  state.setJointGroupPositions(group, joint_pose);
  if (!state.knowsFrameTransform(tool))
  {
    ROS_WARN("NO TOOL TRANSFORM");
    return false;
  }

  if (!state.satisfiesBounds())
  {
    ROS_WARN("Bad state!");
    return false;
  }

  result = state.getFrameTransform(tool);
  return true;
}

bool doIK(moveit::core::RobotState& state,
          const moveit::core::JointModelGroup* group,
          const std::string& group_name,
          const std::string& tool,
          const Eigen::Affine3d& pose,
          const JointConfig& seed,
          JointConfig& result)
{
  state.setJointGroupPositions(group_name, seed);
  if (!state.setFromIK(group, pose, tool))
  {
    return false;
  }
  state.copyJointGroupPositions(group, result);
  return true;
}

void printJointConfig(const JointConfig& a, const char* prefix = "")
{
//  std::ostringstream ss;
//  ss << prefix;
//  for (std::size_t i = 0; i < a.size(); ++i)
//    ss << a[i] << " ";
//  ROS_INFO_STREAM(ss.str());

  ROS_INFO("%s%.4f %.4f %.4f %.4f %.4f %.4f", prefix, a[0], a[1], a[2], a[3], a[4], a[5]);
}

//inline double circularDiff(double a, double b)
//{
//  double diff =
//}

inline bool sameJointConfig(const JointConfig& a, const JointConfig& b, const BijectionVec& bij)
{
  for (std::size_t i = 0; i < FIRST_N; i++)
  {
    if (std::fmod(std::abs(a[bij[i]] - b[bij[i]]), 2*M_PI) > M_PI/4) return false;
  }
  return true;
}

inline bool inJointSet(const JointConfig& c, const JointConfigVec& set, const BijectionVec& bij)
{
  for (std::size_t i = 0; i < set.size(); ++i)
  {
    if (sameJointConfig(c, set[i], bij)) return true;
  }
  return false;
}

bool printJacobian(moveit::core::RobotState& state,
                   const moveit::core::JointModelGroup* group)
{
  Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
  Eigen::MatrixXd jacobian;
  state.getJacobian(group, state.getLinkModel(group->getLinkModelNames().back()),
                               reference_point_position,
                               jacobian);
//  ROS_INFO_STREAM("Jacobian: " << jacobian);
//  ROS_ERROR_STREAM("Determinant: " << jacobian.determinant());

  return std::abs(jacobian.determinant()) > 0.0001;
}



JointConfigVec findSeedStates(moveit::core::RobotStatePtr state, const std::string& group_name, const std::string& tool_frame)
{
  // Get the DOF
  const moveit::core::JointModelGroup* group = state->getJointModelGroup(group_name);

  // Generate a set of valid seed joint configs
  CombinationManager combos (FIRST_N, 4);
  std::vector<CombinationManager::ComboHandle> seed_state_solutions;


//  const static BijectionVec bij = {1,2,0,3,4,5};
  const static BijectionVec bij = {3,5,0,1,2,4};

  // For each joint pose, calculate its 6d pose
  for (std::size_t i = 0; i < combos.count(); i++)
  {
    // For every point, keep track of the IK solutions seen so far
    JointConfigVec iks_seen_so_far; // iks seen so far
    std::vector<CombinationManager::ComboHandle> unique_solution_seeds; // seed states that led to iks seen so far

    auto handle = combos.fromNumber(i);
    JointConfig seed = toJointConfig(handle, bij);
    printJointConfig(seed, "SOURCE SEED: ");

    iks_seen_so_far.push_back(seed);
    unique_solution_seeds.push_back(handle);

    // Compute the pose of this point
    Eigen::Affine3d pose;
    if (!doFK(*state, group, tool_frame, seed, pose))
    {
      ROS_INFO_STREAM("No FK for this point");
      continue;
    }

    // Check the Jacobian to see if the current state of the system
    // is a singularity
    if (!printJacobian(*state, group))
    {
      ROS_WARN("Small jacobian!");
      continue;
    }

    // for every other joint pose, try to IK to this pose; keep only the unique solutions
    for (std::size_t j = 0; j < combos.count(); j++)
    {
      // skip the situation where pose is generated from current seed
      // we already have that ik
      if (i == j) continue;

      auto handle2 = combos.fromNumber(j);
      JointConfig seed2 = toJointConfig(handle2, bij);
      JointConfig ik;

      if (!doIK(*state, group, group_name, tool_frame, pose, seed2, ik))
      {
        continue;
      }

      if (!inJointSet(ik, iks_seen_so_far, bij))
      {
        iks_seen_so_far.push_back(ik);
        unique_solution_seeds.push_back(handle2);
        // push handle to global solutions array as well
        seed_state_solutions.push_back(handle2);
      }

    }

    // Debug Info
    ROS_INFO_STREAM("Calculated " << iks_seen_so_far.size() << " unique IK states this round");
    for (JointConfig::size_type k = 0; k < iks_seen_so_far.size(); ++k)
    {
      printJointConfig(toJointConfig(unique_solution_seeds[k],bij), "seed ");
      printJointConfig(iks_seen_so_far[k], "found: ");
      ROS_INFO_STREAM("");
    }
  }

  // Consolodate the solutions from each round into one set of recommendations
  std::set<CombinationManager::ComboHandle> handle_set;
  handle_set.insert(seed_state_solutions.begin(), seed_state_solutions.end());
  ROS_INFO_STREAM("FOUND: " << handle_set.size());
  return JointConfigVec();
}

} // end namespace seed_search

namespace descartes_moveit
{

MoveitStateAdapter::MoveitStateAdapter():
    sample_iterations_(SAMPLE_ITERATIONS)
{

}

MoveitStateAdapter::MoveitStateAdapter(const moveit::core::RobotState & robot_state, const std::string & group_name,
                                     const std::string & tool_frame, const std::string & world_frame,
                                       const size_t sample_iterations) :
  robot_state_(new moveit::core::RobotState(robot_state)),
  group_name_(group_name),
  tool_frame_(tool_frame),
  world_frame_(world_frame),
  sample_iterations_(sample_iterations),
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

bool MoveitStateAdapter::initialize(const std::string& robot_description, const std::string& group_name,
                                    const std::string& world_frame,const std::string& tcp_frame)
{

  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(robot_description));
  robot_model_ptr_ = robot_model_loader_->getModel();
  robot_state_.reset(new moveit::core::RobotState(robot_model_ptr_));
  planning_scene_.reset(new planning_scene::PlanningScene(robot_model_loader_->getModel()));
  group_name_ = group_name;
  tool_frame_ = tcp_frame;
  world_frame_ = world_frame;

  seed_search::JointConfigVec configs = seed_search::findSeedStates(robot_state_, group_name, tcp_frame);

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
                              tool_frame_))
  {
    robot_state_->copyJointGroupPositions(group_name_, joint_pose);
    if(isInCollision(joint_pose))
    {
      ROS_ERROR_STREAM("Robot is in collision for this pose of the tool '"<<tool_frame_<<"'");
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

bool MoveitStateAdapter::isInCollision(const std::vector<double>& joint_pose) const
{
  bool in_collision = false;
  if(check_collisions_)
  {
    robot_state_->setJointGroupPositions(group_name_, joint_pose);
    in_collision = planning_scene_->isStateColliding(*robot_state_,group_name_);
  }
  return in_collision;
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

    if(isInCollision(joint_pose))
    {
      ROS_INFO_STREAM("Robot is in collision at this joint pose");
      rtn = false;
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

} //descartes_moveit

