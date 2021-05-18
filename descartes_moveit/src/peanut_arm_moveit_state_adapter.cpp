/*
 * author: Achille, adapted from IkFastMoveitStateAdapter
 */

#include "descartes_moveit/peanut_arm_moveit_state_adapter.h"

#include <eigen_conversions/eigen_msg.h>
#include <ros/node_handle.h>

const static std::string default_base_frame = "arm_shoulder_link";
const static std::string default_tool_frame = "end_effector_link";

// Compute the 'joint distance' between two poses
static double distance(const std::vector<double>& a, const std::vector<double>& b)
{
  double cost = 0.0;
  for (size_t i = 0; i < a.size(); ++i)
    cost += std::abs(b[i] - a[i]);
  return cost;
}

// Compute the index of the closest joint pose in 'candidates' from 'target'
static size_t closestJointPose(const std::vector<double>& target, const std::vector<std::vector<double>>& candidates)
{
  size_t closest = 0;  // index into candidates
  double lowest_cost = std::numeric_limits<double>::max();
  for (size_t i = 0; i < candidates.size(); ++i)
  {
    assert(target.size() == candidates[i].size());
    double c = distance(target, candidates[i]);
    if (c < lowest_cost)
    {
      closest = i;
      lowest_cost = c;
    }
  }
  return closest;
}

bool descartes_moveit::PeanutMoveitStateAdapter::initialize(const std::string& robot_description,
                                                            const std::string& group_name,
                                                            const std::string& world_frame,
                                                            const std::string& tcp_frame)
{
  if (!MoveitStateAdapter::initialize(robot_description, group_name, world_frame, tcp_frame))
  {
    return false;
  }

  // Get joint limits
  ros::NodeHandle nh;
  min_pos_.clear();
  max_pos_.clear();
  joint_names_ = {"elevator_joint", "arm_joint_1", "arm_joint_2", "wrist_joint_1", "wrist_joint_2", "wrist_joint_3"};
  for(unsigned int i = 0; i < joint_names_.size(); i++){
    double limit;
    if(nh.getParam("/robot_description_planning/joint_limits/" + joint_names_[i] + "/min_position", limit)){
      min_pos_.push_back(limit);
    }
    else{
      ROS_ERROR_STREAM("Could not find min limits for " << joint_names_[i]);
      return false;
    }
    if(nh.getParam("/robot_description_planning/joint_limits/" + joint_names_[i] + "/max_position", limit)){
     max_pos_.push_back(limit);
    }
    else{
      ROS_ERROR_STREAM("Could not find max limits for " << joint_names_[i]);
      return false;
    }
    joint_limits_dict_[joint_names_[i]] = {min_pos_[i], max_pos_[i]};
  }

  return computeTransforms();
}

bool descartes_moveit::PeanutMoveitStateAdapter::sampleRedundantJoint(std::vector<double>& sampled_joint_vals) const
{
  // all discretized
  double joint_dscrt = 0.1;
  double joint_min = 0.00;
  double joint_max = 3.14;
  size_t steps = std::ceil((joint_max - joint_min) / joint_dscrt);
  for (size_t i = 0; i < steps; i++)
  {
    sampled_joint_vals.push_back(joint_min + joint_dscrt * i);
  }
  sampled_joint_vals.push_back(joint_max);

  return true;
}

bool descartes_moveit::PeanutMoveitStateAdapter::getAllIK(const Eigen::Isometry3d& pose,
                                                          std::vector<std::vector<double>>& joint_poses) const
{
  joint_poses.clear();

  // Transform input pose (given in reference frame world to be reached by the tip, so transform to base and tool)
  // math: pose = H_w_b * tool_pose * H_tip_tool0 (pose = H_w_pose; tool_pose = H_b_tip, with tip st tool is at pose)
  // Eigen::Isometry3d tool_pose = world_to_base_.frame_inv * pose * tool0_to_tip_.frame;
  // for now disable the above since we're just going to feed it in exactly what it needs:
  Eigen::Isometry3d tool_pose = pose;

  // bool ik(const Eigen::Isometry3d &pose, std::vector<std::vector<double> > &qs,
  //           const std::vector<std::string> joint_names, const std::vector<float> min_pos, const std::vector<float> max_pos,
  //           const bool check_limits=true,
  //           const bool check_cord_wrap=false,
  //           const bool debug=false);

  std::vector<std::vector<double>> potential_joint_configs;
  bool success = arm_kinematics::ik(tool_pose, potential_joint_configs, joint_names_, min_pos_, max_pos_, true, true);

  for(auto& joint_config : potential_joint_configs){
    if(isValid(joint_config)){
      joint_poses.push_back(std::move(joint_config));
    }
  }
  if (joint_poses.size() == 0){
    ROS_DEBUG_STREAM_THROTTLE(0.25, "Could not find ik");
  }
  return joint_poses.size() > 0;
}

bool descartes_moveit::PeanutMoveitStateAdapter::getIK(const Eigen::Isometry3d& pose,
                                                       const std::vector<double>& seed_state,
                                                       std::vector<double>& joint_pose) const
{
  // Descartes Robot Model interface calls for 'closest' point to seed position
  std::vector<std::vector<double>> joint_poses;
  if (!getAllIK(pose, joint_poses))
    return false;
  // Find closest joint pose; getAllIK() does isValid checks already
  joint_pose = joint_poses[closestJointPose(seed_state, joint_poses)];
  return true;
}

bool descartes_moveit::PeanutMoveitStateAdapter::getFK(const std::vector<double>& joint_pose,
                                                       Eigen::Isometry3d& pose) const
{
  const auto& solver = joint_group_->getSolverInstance();

  std::vector<std::string> tip_frame = { solver->getTipFrame() };
  std::vector<geometry_msgs::Pose> output;

  if (!isValid(joint_pose))
    return false;

  // tip frame is end-effector-link (Achille)
  if (!solver->getPositionFK(tip_frame, joint_pose, output))
    return false;

  tf::poseMsgToEigen(output[0], pose);  // pose in frame of IkFast base
  // pose = world_to_base_.frame * pose * tool0_to_tip_.frame_inv;
  return true;
}

void descartes_moveit::PeanutMoveitStateAdapter::setState(const moveit::core::RobotState& state)
{
  descartes_moveit::MoveitStateAdapter::setState(state);
  computeTransforms();
}

bool descartes_moveit::PeanutMoveitStateAdapter::computeTransforms()
{
  // look up the IKFast base and tool frame
  ros::NodeHandle nh;
  std::string robot_base_frame, peanut_tool_frame;
  nh.param<std::string>("peanut_base_frame", robot_base_frame, default_base_frame);
  nh.param<std::string>("peanut_tool_frame", peanut_tool_frame, default_tool_frame);

  if (!robot_state_->knowsFrameTransform(robot_base_frame))
  {
    CONSOLE_BRIDGE_logError("PeanutMoveitStateAdapter: Cannot find transformation to frame '%s' in group '%s'.",
             robot_base_frame.c_str(), group_name_.c_str());
    return false;
  }

  if (!robot_state_->knowsFrameTransform(peanut_tool_frame))
  {
    CONSOLE_BRIDGE_logError("PeanutMoveitStateAdapter: Cannot find transformation to frame '%s' in group '%s'.",
             peanut_tool_frame.c_str(), group_name_.c_str());
    return false;
  }

  // calculate frames
  tool0_to_tip_ = descartes_core::Frame(robot_state_->getFrameTransform(tool_frame_).inverse() *
                                        robot_state_->getFrameTransform(peanut_tool_frame));

  world_to_base_ = descartes_core::Frame(world_to_root_.frame * robot_state_->getFrameTransform(robot_base_frame));

  CONSOLE_BRIDGE_logInform("PeanutMoveitStateAdapter: initialized with IKFast tool frame '%s' and base frame '%s'.",
            peanut_tool_frame.c_str(), robot_base_frame.c_str());
  return true;
}

bool descartes_moveit::PeanutMoveitStateAdapter::hasNaN(const std::vector<double>& joint_pose) const{
  for(const auto&val : joint_pose){
    if(std::isnan(val)){
      return true;
    }
  }
  return false;
}

void descartes_moveit::PeanutMoveitStateAdapter::setCollisionLinks(std::vector<std::string> arm_links, std::vector<std::string> robot_links){
  collision_arm_links_ = arm_links;
  collision_robot_links_ = robot_links;

  std::string msg = "collision_arm_links_ : ";
  for(const auto& name: collision_arm_links_){
    msg = msg + "" + name + ",";
  }
  ROS_INFO_STREAM(msg);

  msg = "collision_robot_links_ : ";
  for(const auto& name: collision_robot_links_){
    msg = msg + "" + name + ",";
  }
  ROS_INFO_STREAM(msg);
}

bool descartes_moveit::PeanutMoveitStateAdapter::isValid(const std::vector<double>& joint_pose) const{
  // Logical check on input sizes
  if (joint_group_->getActiveJointModels().size() != joint_pose.size())
  {
    CONSOLE_BRIDGE_logError("Size of joint pose: %lu doesn't match robot state variable size: %lu",
             static_cast<unsigned long>(joint_pose.size()),
             static_cast<unsigned long>(joint_group_->getActiveJointModels().size()));
    return false;
  }

  return !hasNaN(joint_pose) && isInLimits(joint_pose) && !isInCollision(joint_pose);
}

bool descartes_moveit::PeanutMoveitStateAdapter::isInLimits(const std::vector<double> &joint_pose) const{
 const std::vector<const moveit::core::JointModel*> joints = joint_group_->getActiveJointModels();
 int i = 0;
 for(const moveit::core::JointModel* j : joints){
   std::string name = j->getName();
   const double min_limit = joint_limits_dict_.at(name)[0];
   const double max_limit = joint_limits_dict_.at(name)[1];
   if(joint_pose[i] < min_limit || joint_pose[i] > max_limit){
     return false;
   }
   i++;
 }
 return true;
}

bool descartes_moveit::PeanutMoveitStateAdapter::updatePlanningScene(planning_scene::PlanningScenePtr ps){
  // Initialize planning scene
  planning_scene_ = ps;

  // Update ACM
  acm_ = planning_scene_->getAllowedCollisionMatrix();
  // Disable all collision checking
  // acm_.setEntry(true);
  // Collision check selected arm links with selected robot links
  // acm_.setEntry(collision_robot_links_, collision_arm_links_, false);

  // Initialize collision request message.
  // Setting this to false could descrease collision checking speed
  collision_request_.contacts = true;
  return true;
}

bool descartes_moveit::PeanutMoveitStateAdapter::isInCollision(const std::vector<double>& joint_pose) const
{
  bool in_collision = false;

  if (check_collisions_)
  {
    collision_detection::CollisionResult collision_result;

    moveit::core::RobotState state = planning_scene_->getCurrentStateNonConst();
    state.setJointGroupPositions(joint_group_, joint_pose);

    planning_scene_->checkCollision(collision_request_, collision_result, state, acm_);
    in_collision = collision_result.collision;

    if (in_collision){
      collision_detection::CollisionResult::ContactMap::const_iterator it;
      for ( it = collision_result.contacts.begin(); it != collision_result.contacts.end(); it++ )
      {
        ROS_WARN_STREAM_THROTTLE(0.5, "Contact between: "<<it->first.first.c_str()<<" and "<<it->first.second.c_str());
      }
    }

  }

  return in_collision;
}
