/*
 * Link moveit's robot model with Descartes' robot model and plug in our custom IK solver
 */

#ifndef PEANUT_ARM_MOVEIT_STATE_ADAPTER_H
#define PEANUT_ARM_MOVEIT_STATE_ADAPTER_H

#include "descartes_moveit/moveit_state_adapter.h"
#include <peanut_kinematics/arm_kinematics.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <peanut_common_util/peanut_common_util.h>

namespace descartes_moveit
{
class PeanutMoveitStateAdapter : public descartes_moveit::MoveitStateAdapter
{
public:
  virtual ~PeanutMoveitStateAdapter()
  {
  }

  virtual bool initialize(const std::string& robot_description, const std::string& group_name,
                          const std::string& world_frame, const std::string& tcp_frame);

  virtual bool sampleRedundantJoint(std::vector<double>& sampled_joint_vals) const;

  virtual bool getAllIK(const Eigen::Isometry3d& pose, std::vector<std::vector<double> >& joint_poses) const;

  virtual bool getIK(const Eigen::Isometry3d& pose, const std::vector<double>& seed_state,
                     std::vector<double>& joint_pose) const;

  virtual bool getFK(const std::vector<double>& joint_pose, Eigen::Isometry3d& pose) const;

  virtual bool isValid(const std::vector<double>& joint_pose) const;

  virtual bool updatePlanningScene(planning_scene::PlanningScenePtr ps);

  virtual bool isInLimits(const std::vector<double> &joint_pose) const;

  /**
   * @brief Sets the internal state of the robot model to the argument. For the IKFast impl,
   * it also recomputes the transformations to/from the IKFast reference frames.
   */
  void setState(const moveit::core::RobotState& state);

  bool hasNaN(const std::vector<double> &joint_pose) const;

protected:
  bool computeTransforms();

  bool getAllIKSprayer(const Eigen::Isometry3d& pose, std::vector<std::vector<double> >& joint_poses) const;
  bool getAllIKBrushContact(const Eigen::Isometry3d& pose, std::vector<std::vector<double> >& joint_poses) const;

  virtual bool isInCollision(const std::vector<double>& joint_pose) const;

  /**
   * Update the collision_arm_links_ and collision_arm_robot_links
   */
  virtual void setCollisionLinks(std::vector<std::string> arm_links, std::vector<std::string> robot_links);

  /**
   * The IKFast implementation commonly solves between 'base_link' of a robot
   * and 'tool0'. We will commonly want to take advantage of an additional
   * fixed transformation from the robot flange, 'tool0', to some user defined
   * tool. This prevents the user from having to manually adjust tool poses to
   * account for this.
   */
  descartes_core::Frame tool0_to_tip_;

  std::string tool_frame_ = "";

  /**
   * Likewise this parameter is used to accomodate transformations between the base
   * of the IKFast solver and the base of the MoveIt move group.
   */
  descartes_core::Frame world_to_base_;

  collision_detection::AllowedCollisionMatrix acm_;
  collision_detection::CollisionRequest collision_request_;
  std::string octomap_link_ = "<octomap>";
  std::vector<std::string> collision_arm_links_ = {"arm_upper_link", "arm_lower_link"};
  std::vector<std::string> collision_robot_links_ = {"tower_link"};
  std::vector<double> min_pos_;
  std::vector<double> max_pos_;
  std::vector<std::string> joint_names_;
  std::map<std::string, std::vector<double>> joint_limits_dict_;
};

}  // end namespace 'descartes_moveit'
#endif
