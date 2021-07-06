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

  ROS_INFO_STREAM("PeanutMoveitStateAdapter::initialize: " << group_name << " " << "tcp_frame " << tcp_frame);
  // can initialize here to set spray vs wipe vs other?
  this->tool_frame_ = tcp_frame;

  // Get joint limits
  ros::NodeHandle nh;
  std::vector<std::vector<double>> joint_limits;
  min_pos_.clear();
  max_pos_.clear();
  joint_names_ = peanut_common_util::getArmElevatorJointNames();
  if(!peanut_common_util::getArmElevatorLimits(joint_limits)){
    ROS_ERROR("Could not find get arm elevator limits");
    return false;
  }

  for(unsigned int i = 0; i < joint_names_.size(); i++){
    min_pos_.push_back(joint_limits[i][0]);
    max_pos_.push_back(joint_limits[i][1]);
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
  if (this->tool_frame_ == "sprayer_link") {
    return this->getAllIKSprayer(pose, joint_poses);
  }
  if (this->tool_frame_ == "brush_contact_link") {
    return this->getAllIKBrushContact(pose, joint_poses);
  }

  // default implemenetation
  joint_poses.clear();

  // bool ik(const Eigen::Isometry3d &pose, std::vector<std::vector<double> > &qs,
  //           const std::vector<std::string> joint_names, const std::vector<float> min_pos, const std::vector<float> max_pos,
  //           const bool check_limits=true,
  //           const bool check_cord_wrap=false,
  //           const bool debug=false);

  // IK is done in eff pose
  const auto raw_pos = pose.translation();
  ROS_DEBUG_STREAM_THROTTLE(0.25, "Raw pos " << raw_pos[0] << " " << raw_pos[1] << " " << raw_pos[2]);

  const auto pos = tool0_to_tip_.frame.translation();

  const auto rot = tool0_to_tip_.frame.rotation();
  Eigen::Quaterniond q_eff_to_sprayer(rot);

  Eigen::Isometry3d eff_pose = pose * tool0_to_tip_.frame_inv;
  const auto eff_rot = eff_pose.rotation();
  Eigen::Quaterniond q_eff(eff_rot);

  ROS_DEBUG_STREAM_THROTTLE(0.25, "Eff Q " << q_eff.x() << " " << q_eff.y() << " " << q_eff.z() << " " << q_eff.w());
  const auto eff_pos = eff_pose.translation();
  ROS_DEBUG_STREAM_THROTTLE(0.25, "Eff pos " << eff_pos[0] << " " << eff_pos[1] << " " << eff_pos[2]);

  std::vector<std::vector<double>> potential_joint_configs;
  bool success = arm_kinematics::ik(eff_pose, potential_joint_configs, joint_names_, min_pos_, max_pos_, true, false);

  if (!success){
    ROS_WARN_STREAM("Could not find ik");
    // this is not necessarily a fatal error, as Descartes will try many orientations
    return false;
  }

  for(auto& joint_config : potential_joint_configs){
    if(isValid(joint_config)){
      joint_poses.push_back(std::move(joint_config));
    }
  }
  if (joint_poses.size() == 0){
    ROS_WARN_STREAM_THROTTLE(0.25, "getAllIK(): Invalid joints");
  }
  return joint_poses.size() > 0;
}

bool descartes_moveit::PeanutMoveitStateAdapter::getAllIKSprayer(const Eigen::Isometry3d& pose,
                                                          std::vector<std::vector<double>>& joint_poses) const
{
  joint_poses.clear();

  // bool ik(const Eigen::Isometry3d &pose, std::vector<std::vector<double> > &qs,
  //           const std::vector<std::string> joint_names, const std::vector<float> min_pos, const std::vector<float> max_pos,
  //           const bool check_limits=true,
  //           const bool check_cord_wrap=false,
  //           const bool debug=false);

  // IK is done in eff pose
  const auto raw_pos = pose.translation();
  ROS_DEBUG_STREAM_THROTTLE(0.25, "Raw pos " << raw_pos[0] << " " << raw_pos[1] << " " << raw_pos[2]);

  const auto pos = tool0_to_tip_.frame.translation();
  ROS_DEBUG_STREAM_THROTTLE(0.25, "Eff to sprayer " << pos[0] << " " << pos[1] << " " << pos[2]);

  const auto rot = tool0_to_tip_.frame.rotation();
  Eigen::Quaterniond q_eff_to_sprayer(rot);
  ROS_DEBUG_STREAM_THROTTLE(0.25, "Q Eff to sprayer " << q_eff_to_sprayer.x() << " " << q_eff_to_sprayer.y() << " " << q_eff_to_sprayer.z() << " " << q_eff_to_sprayer.w());

  Eigen::Isometry3d eff_pose = pose * tool0_to_tip_.frame_inv;
  const auto eff_rot = eff_pose.rotation();
  Eigen::Quaterniond q_eff(eff_rot);

  ROS_DEBUG_STREAM_THROTTLE(0.25, "Eff Q " << q_eff.x() << " " << q_eff.y() << " " << q_eff.z() << " " << q_eff.w());
  const auto eff_pos = eff_pose.translation();
  ROS_DEBUG_STREAM_THROTTLE(0.25, "Eff pos " << eff_pos[0] << " " << eff_pos[1] << " " << eff_pos[2]);

  std::vector<std::vector<double>> potential_joint_configs;
  bool success = arm_kinematics::ik(eff_pose, potential_joint_configs, joint_names_, min_pos_, max_pos_, true, false);

  if (!success){
    ROS_WARN_STREAM("Could not find ik");
    // this is not necessarily a fatal error, as Descartes will try many orientations
    return false;
  }

  for(auto& joint_config : potential_joint_configs){
    if(isValid(joint_config)){
      joint_poses.push_back(std::move(joint_config));
    }
  }
  if (joint_poses.size() == 0){
    ROS_DEBUG_STREAM_THROTTLE(0.25, "getAllIKSprayer(): Invalid joints");
  }
  return joint_poses.size() > 0;
}

bool descartes_moveit::PeanutMoveitStateAdapter::getAllIKBrushContact(const Eigen::Isometry3d& pose,
                                                          std::vector<std::vector<double>>& joint_poses) const
{
  double brush_pitch = 10.0 * M_PI / 180.0; // MUST be set to match table_plannner TODO: should be in MoveArmGoal
  ros::NodeHandle nh;
  if (!nh.getParam("brush_pitch", brush_pitch)) { // xx!! should be /brush_pitch ?
    ROS_DEBUG_STREAM_THROTTLE(1.0, "Unable to load brush_pitch param. Using " << brush_pitch);
  }

  // const auto contact_pos = pose.translation();
  const auto rot = pose.rotation();
  Eigen::Vector3d rpy = rot.eulerAngles(0, 1, 2);
  // ROS_INFO_STREAM("brush rpy " << rpy.x() << " " << rpy.y() << " " << rpy.z());
  Eigen::Quaterniond q_yaw(Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ())); // effector yaw
  // brush_yaw = pose roll
  const double brush_yaw = rpy.x();

  // implement q_eff_for_brush_yaw from arm_kinematics.py, probably not efficient
  const double deg45 = 45.0 * M_PI / 180.0;
  Eigen::Quaterniond q_axis_base(Eigen::AngleAxisd(deg45, Eigen::Vector3d::UnitY()));
  Eigen::Quaterniond q_axis_to_eff(Eigen::AngleAxisd(deg45, Eigen::Vector3d::UnitY()));
  Eigen::Quaterniond q_axis_to_down(Eigen::AngleAxisd(deg45, Eigen::Vector3d::UnitY()));
  Eigen::Vector3d facing_axis(cos(brush_yaw), sin(brush_yaw), 0.0);
  Eigen::Vector3d left_axis = Eigen::Vector3d::UnitZ().cross(facing_axis);
  Eigen::Quaterniond q_tilt(Eigen::AngleAxisd(brush_pitch, left_axis));
  Eigen::Quaterniond q_axis = q_tilt * q_axis_to_down * q_axis_base;
  Eigen::Quaterniond q_eff = q_axis * q_axis_to_eff;
  q_eff = q_yaw * q_eff;

  // implement compute_eff_to_brush_offsets from TablePlanner.py
  const double BRUSH_DISC_RADIUS = 0.22;
  const double BRUSH_AXIS_OFFSET = 0.01; // brush is slightly down the axis
  const Eigen::Vector3d EFF_TO_AXIS(0.0175, 0.0, 0.0); // effector is slightly above brush axis
  Eigen::Vector3d eff_to_axis = q_eff * EFF_TO_AXIS;
  const Eigen::Vector3d AXIS_DIR(0.707, 0.0, 0.707); //axis is midway between effector's Z and X
  Eigen::Vector3d axis_dir = q_eff * AXIS_DIR;
  Eigen::Vector3d side_axis = axis_dir.cross(Eigen::Vector3d::UnitZ());
  side_axis.normalize();
  Eigen::Vector3d brush_axis = axis_dir.cross(side_axis);
  if (brush_axis.z() > 0.0) {
    brush_axis *= -1.0;
  }
  auto eff_to_brush = eff_to_axis + BRUSH_DISC_RADIUS * brush_axis + BRUSH_AXIS_OFFSET * axis_dir;

  // ROS_INFO_STREAM("q_eff " << q_eff.x() << " " << q_eff.y() << " " << q_eff.z() << " " << q_eff.w());
  Eigen::Matrix3d eff_rot(q_eff);
  Eigen::Vector3d eff_trans(pose.translation() - eff_to_brush);
  Eigen::Vector3d iscale(1.0, 1.0, 1.0); // identity scale
  Eigen::Isometry3d eff_pose;
  eff_pose.fromPositionOrientationScale(eff_trans, eff_rot, iscale);
  // ROS_INFO_STREAM("eff_pose.translation " << eff_pose.translation().x() << " " << eff_pose.translation().y() << " " << eff_trans.x() << " " << eff_trans.y());
  Eigen::Quaterniond q_test(eff_pose.rotation());
  // ROS_INFO_STREAM("eff_pose.rotation " << q_test.x() << " " << q_test.y() << " " << q_test.z() << " " << q_test.w());

  joint_poses.clear();
  std::vector<std::vector<double>> potential_joint_configs;
  bool success = arm_kinematics::ik(eff_pose, potential_joint_configs, joint_names_, min_pos_, max_pos_, true, false);

  if (!success){
    // ROS_WARN_STREAM("Could not find ik");
    // this is not necessarily a fatal error, as Descartes will try many orientations
    return false;
  }

  for(auto& joint_config : potential_joint_configs){
    // ROS_INFO_STREAM("testing joint config " << joint_config.size());
    if(isValid(joint_config)){
      joint_poses.push_back(std::move(joint_config));
    }
  }
  if (joint_poses.size() == 0){
    ROS_DEBUG_STREAM_THROTTLE(0.25, "getAllIKBrushContact(): Invalid joints");
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

  ROS_WARN_STREAM_THROTTLE(0.25, "getFK tip_frame " << tip_frame[0]);

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
  tool0_to_tip_ = descartes_core::Frame(robot_state_->getFrameTransform(peanut_tool_frame).inverse() *
                                        robot_state_->getFrameTransform(tool_frame_));
  ROS_INFO_STREAM("tool_frame_ " << tool_frame_ << " peanut_tool_frame " << peanut_tool_frame);

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
    for (const auto& model : joint_group_->getActiveJointModels()) {
      ROS_INFO_STREAM(model->getName());
    }
    CONSOLE_BRIDGE_logError("(peanut) Size of joint pose: %lu doesn't match robot state variable size: %lu",
             static_cast<unsigned long>(joint_pose.size()),
             static_cast<unsigned long>(joint_group_->getActiveJointModels().size()));
    return false;
  }

  if (hasNaN(joint_pose)) {
    ROS_WARN_STREAM("NaN joints shouldn't happen");
    return false;
  }
  if (!isInLimits(joint_pose)) {
    ROS_DEBUG_STREAM("invalid joints = " << joint_pose[0] << " " << joint_pose[1] << " " << joint_pose[2]
                    << " " << joint_pose[3] << " " << joint_pose[4] << " " << joint_pose[5]);
    return false;
  }
  if (isInCollision(joint_pose)) {
    return false;
  }
  return true;
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
