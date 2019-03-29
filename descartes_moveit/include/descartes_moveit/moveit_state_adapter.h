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

#ifndef MOVEIT_STATE_ADAPTER_H_
#define MOVEIT_STATE_ADAPTER_H_

#include "descartes_core/robot_model.h"
#include "descartes_trajectory/cart_trajectory_pt.h"

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <string>


namespace descartes_moveit
{
/**
 * @brief MoveitStateAdapter adapts the MoveIt RobotState to the Descartes RobotModel interface
 */
class MoveitStateAdapter : public descartes_core::RobotModel
{
public:
  MoveitStateAdapter();

  virtual ~MoveitStateAdapter()
  {
  }

  virtual bool initialize(const std::string &robot_description, const std::string &group_name,
                          const std::string &world_frame, const std::string &tcp_frame);

  virtual bool initialize(planning_scene_monitor::PlanningSceneMonitorPtr& psm, const std::string &group_name,
                          const std::string &world_frame, const std::string &tcp_frame);

  virtual bool getIK(const Eigen::Isometry3d &pose, const std::vector<double> &seed_state,
                     std::vector<double> &joint_pose) const;

  virtual bool getAllIK(const Eigen::Isometry3d &pose, std::vector<std::vector<double> > &joint_poses) const;

  virtual bool getFK(const std::vector<double> &joint_pose, Eigen::Isometry3d &pose) const;

  virtual bool isValid(const std::vector<double> &joint_pose) const;

  virtual bool isValid(const Eigen::Isometry3d &pose) const;

  virtual int getDOF() const;

  virtual bool isValidMove(const double* from_joint_pose, const double* to_joint_pose,
                           double dt) const;

  virtual std::vector<double> getJointVelocityLimits() const override;
  /**
   * @brief Set the initial states used for iterative inverse kineamtics
   * @param seeds Vector of vector of doubles representing joint positions.
   *              Be sure that it's sized correctly for the DOF.
   */
  void setSeedStates(const std::vector<std::vector<double> > &seeds)
  {
    seed_states_ = seeds;
  }

  /**
   * @brief Retrieves the initial seed states used by iterative inverse kinematic solvers
   */
  const std::vector<std::vector<double> > &getSeedStates() const
  {
    return seed_states_;
  }

  /**
   * @brief Returns the underlying moveit state object so it can be used to generate seeds
   */
  moveit::core::RobotStatePtr getState()
  {
    return robot_state_;
  }

  /**
   * @brief Copies the internal state of 'state' into this model. Useful for initializing the
   *        value of joints that are not part of the active move group. Should be called after
   *        'initialize()'.
   */
  void setState(const moveit::core::RobotState &state);

protected:
  /**
   * Gets IK solution (assumes robot state is pre-seeded)
   * @param pose Affine pose of TOOL in WOBJ frame
   * @param joint_pose Solution (if function successful).
   * @return
   */
  bool getIK(const Eigen::Isometry3d &pose, std::vector<double> &joint_pose) const;

  /**
   * TODO: Checks for collisions at this joint pose. The setCollisionCheck(true) must have been
   * called previously in order to enable collision checks, otherwise it will return false.
   * @param joint_pose the joint values at which check for collisions will be made
   */
  bool isInCollision(const std::vector<double>& joint_pose) const;

  /**
   * @brief Checks to see if the given joint_pose state is inside the bounds for the initialized
   * robot model's active joints.
   * @param joint_pose The pose to check; should be the same length as your groups active num active joints
   * @return true if the @e joint_pose is in bounds, false otherwise
   */
  bool isInLimits(const std::vector<double>& joint_pose) const;

  /**
   * @brief Updates the robot_state_ variable with the most recent joint states from the psm.
   */
  bool getCurrentRobotState() const;

  /**
   * Maximum joint velocities (rad/s) for each joint in the chain. Used for checking in
   * `isValidMove()`
   */
  std::vector<double> velocity_limits_;

  mutable moveit::core::RobotStatePtr robot_state_;

  const moveit::core::JointModelGroup *joint_group_;

  /**
   * @brief Vector of starting configurations for the numerical solver
   */
  std::vector<std::vector<double> > seed_states_;

  /**
   * @brief Planning group name
   */
  std::string group_name_;

  /**
   * @brief Tool frame name
   */
  std::string tool_frame_;

  /**
   * @brief Work object/reference frame name
   */
  std::string world_frame_;

  /**
   * @brief convenient transformation frame
   */
  descartes_core::Frame world_to_root_;

  /**
   * @brief Planning scene monitor (used to update internal planning scene)
   */
  mutable planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
};

}  // descartes_moveit

#endif /* MOVEIT_STATE_ADAPTER_H_ */
