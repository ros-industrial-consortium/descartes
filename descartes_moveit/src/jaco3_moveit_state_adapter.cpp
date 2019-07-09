/*
 * author: Achille, adapted from IkFastMoveitStateAdapter
 */

#include "descartes_moveit/jaco3_moveit_state_adapter.h"

#include <eigen_conversions/eigen_msg.h>
#include <ros/node_handle.h>

const static std::string default_base_frame = "base_link";
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

bool descartes_moveit::Jaco3MoveitStateAdapter::initialize(const std::string& robot_description,
                                                            const std::string& group_name,
                                                            const std::string& world_frame,
                                                            const std::string& tcp_frame)
{
  if (!MoveitStateAdapter::initialize(robot_description, group_name, world_frame, tcp_frame))
  {
    return false;
  }

  return computeJaco3Transforms();
}

bool descartes_moveit::Jaco3MoveitStateAdapter::sampleRedundantJoint(std::vector<double>& sampled_joint_vals) const
{
  // all discretized
  double joint_dscrt = 0.3;
  double joint_min = -3.14;
  double joint_max = 3.14;
  size_t steps = std::ceil((joint_max - joint_min) / joint_dscrt);
  for (size_t i = 0; i < steps; i++)
  {
    sampled_joint_vals.push_back(joint_min + joint_dscrt * i);
  }
  sampled_joint_vals.push_back(joint_max);

  return true;
}

bool descartes_moveit::Jaco3MoveitStateAdapter::getAllIK(const Eigen::Affine3d& pose,
                                                          std::vector<std::vector<double>>& joint_poses) const
{
  joint_poses.clear();
//  const auto& solver = joint_group_->getSolverInstance();

  // Transform input pose (given in reference frame world to be reached by the tip, so transform to base and tool)
  // math: pose = H_w_b * tool_pose * H_tip_tool0 (pose = H_w_pose; tool_pose = H_b_tip, with tip st tool is at pose)
//  Eigen::Affine3d tool_pose = world_to_base_.frame_inv * pose * tool0_to_tip_.frame;
  // for now disable the above since we're just going to feed it in exactly what it needs:
  Eigen::Affine3d tool_pose = pose;

//  // convert to geometry_msgs ...
//  geometry_msgs::Pose geometry_pose;
//  tf::poseEigenToMsg(tool_pose, geometry_pose);
//  std::vector<geometry_msgs::Pose> poses = { geometry_pose };
//
//  std::vector<double> dummy_seed(getDOF(), 0.0);
//  std::vector<std::vector<double>> joint_results;
//  kinematics::KinematicsResult result;
//  kinematics::KinematicsQueryOptions options;  // defaults are reasonable as of Indigo
//
//  if (!solver->getPositionIK(poses, dummy_seed, joint_results, result, options))
//  {
//    return false;
//  }

  // sample redundant joints
  std::vector<double> sampled_joint_vals;
  if (!sampleRedundantJoint(sampled_joint_vals))
  {
    return false;
  }

  // find IK solutions for each sampled redundant parameter
  int nb_solutions;
  Eigen::Matrix4d T = pose.matrix();
  std::vector<std::vector<double>> joint_results;

  for (auto& redundant_param : sampled_joint_vals)
  {
    Eigen::Matrix<double, 16, 7> q_solns;
    nb_solutions = jaco3_kinematics::ik_with_redundant_param(T, q_solns, redundant_param);

    // convert and add to solutions matrix
    for (int row=0; row<nb_solutions; ++row)
    {
      std::vector<double> intermed;
      for (unsigned int col = 0; col < 7; col++)
      {
        intermed.push_back(q_solns(row, col));
      }
    joint_results.push_back(intermed);
    }

//    std::cout << "IK res: " << q_solns.row(row) << std::endl;
//    // print out the vector
//    std::cout << "converted: ";
//    for (auto i = intermed.begin(); i != intermed.end(); ++i)
//      std::cout << *i << ' ';
//    std::cout << std::endl;
  }

  for (auto& sol : joint_results)
  {
    if (isValid(sol))
    {
      joint_poses.push_back(std::move(sol));
       // test
//      Eigen::Affine3d res_pose;
//      if (!getFK(sol, res_pose))
//      {
//        CONSOLE_BRIDGE_logError("FK failed on joints that should have worked!");
//      }
//      else
//      {
//        std::cout << res_pose.translation() << std::endl;
//      }
    }
  }

  return joint_poses.size() > 0;
}

bool descartes_moveit::Jaco3MoveitStateAdapter::getIK(const Eigen::Affine3d& pose,
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

bool descartes_moveit::Jaco3MoveitStateAdapter::getFK(const std::vector<double>& joint_pose,
                                                       Eigen::Affine3d& pose) const
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
//  pose = world_to_base_.frame * pose * tool0_to_tip_.frame_inv;
  return true;
}

void descartes_moveit::Jaco3MoveitStateAdapter::setState(const moveit::core::RobotState& state)
{
  descartes_moveit::MoveitStateAdapter::setState(state);
  computeJaco3Transforms();
}

bool descartes_moveit::Jaco3MoveitStateAdapter::computeJaco3Transforms()
{
  // look up the IKFast base and tool frame
  ros::NodeHandle nh;
  std::string jaco3_base_frame, jaco3_tool_frame;
  nh.param<std::string>("jaco3_base_frame", jaco3_base_frame, default_base_frame);
  nh.param<std::string>("jaco3_tool_frame", jaco3_tool_frame, default_tool_frame);

  if (!robot_state_->knowsFrameTransform(jaco3_base_frame))
  {
    CONSOLE_BRIDGE_logError("Jaco3MoveitStateAdapter: Cannot find transformation to frame '%s' in group '%s'.",
             jaco3_base_frame.c_str(), group_name_.c_str());
    return false;
  }

  if (!robot_state_->knowsFrameTransform(jaco3_tool_frame))
  {
    CONSOLE_BRIDGE_logError("Jaco3MoveitStateAdapter: Cannot find transformation to frame '%s' in group '%s'.",
             jaco3_tool_frame.c_str(), group_name_.c_str());
    return false;
  }

  // calculate frames
  tool0_to_tip_ = descartes_core::Frame(robot_state_->getFrameTransform(tool_frame_).inverse() *
                                        robot_state_->getFrameTransform(jaco3_tool_frame));

  world_to_base_ = descartes_core::Frame(world_to_root_.frame * robot_state_->getFrameTransform(jaco3_base_frame));

  CONSOLE_BRIDGE_logInform("Jaco3MoveitStateAdapter: initialized with IKFast tool frame '%s' and base frame '%s'.",
            jaco3_tool_frame.c_str(), jaco3_base_frame.c_str());
  return true;
}