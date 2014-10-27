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

#ifndef MOVEIT_STATE_ADPATER_H_
#define MOVEIT_STATE_ADPATER_H_

#include "descartes_trajectory_planning/robot_model.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/kinematics_base/kinematics_base.h"
#include <string>

namespace descartes_moveit
{

/**@brief MoveitStateAdapter adapts the MoveIt RobotState to the Descartes RobotModel interface
 *
 */
class MoveitStateAdapter : public descartes::RobotModel
{
public:

  /**
   * Constructor for Moveit state adapters (implements Descartes robot model interface)
   * @param robot_state robot state object utilized for kinematic/dynamic state checking
   * @param group_name planning group name
   * @param tool_frame tool frame name
   * @param wobj_frame work object frame name
   */
  MoveitStateAdapter(const moveit::core::RobotState & robot_state, const std::string & group_name,
                    const std::string & tool_frame, const std::string & wobj_frame,
                     const size_t sample_iterations = 10);
  virtual ~MoveitStateAdapter()
  {
  }
  ;

  virtual bool getIK(const Eigen::Affine3d &pose, const std::vector<double> &seed_state,
                     std::vector<double> &joint_pose) const;

  virtual bool getAllIK(const Eigen::Affine3d &pose, std::vector<std::vector<double> > &joint_poses) const;

  virtual bool getFK(const std::vector<double> &joint_pose, Eigen::Affine3d &pose) const;

  virtual bool isValid(const std::vector<double> &joint_pose) const;

  virtual bool isValid(const Eigen::Affine3d &pose) const;

protected:

  /**
   * @brief Default constructor hidden
   */
  MoveitStateAdapter()
  {
  }
  ;

  /**
   * Gets IK solution (assumes robot state is pre-seeded)
   * @param pose Affine pose of TOOL in WOBJ frame
   * @param joint_pose Solution (if function successful).
   * @return
   */
  bool getIK(const Eigen::Affine3d &pose, std::vector<double> &joint_pose) const;

  /**
   * @brief Pointer to moveit robot state (mutable object state is reset with
   * each function call
   */
  mutable moveit::core::RobotStatePtr robot_state_;

  /**
   * @brief Planning group name
   */
  std::string group_name_;

  /**
   * @brief Tool frame name
   */
  std::string tool_base_;

  /**
   * @brief Work object/reference frame name
   */
  std::string wobj_base_;

  /**
   * @brief Joint solution sample iterations for returning "all" joints
   */
  size_t sample_iterations_;

};

} //descartes_moveit

#endif /* MOVEIT_STATE_ADPATER_H_ */
