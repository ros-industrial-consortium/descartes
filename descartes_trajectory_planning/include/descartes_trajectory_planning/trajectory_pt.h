/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
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
/*
 * trajectory_pt.h
 *
 *  Created on: Jun 5, 2014
 *      Author: Dan Solomon
 */

#ifndef TRAJECTORY_PT_H_
#define TRAJECTORY_PT_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include <moveit/robot_state/robot_state.h>
#include <vector>
#include "descartes_trajectory_planning/robot_model.h"
#include "descartes_trajectory_planning/trajectory_pt_transition.h"


namespace descartes
{

/**@brief Frame is a wrapper for an affine frame transform.
 * Frame inverse can also be stored for increased speed in downstream calculations.
 */
struct Frame
{
  Frame(){};
  Frame(const Eigen::Affine3d &a):
    frame(a), frame_inv(a.inverse()) {};
  Frame(const Frame &a):
    frame(a.frame), frame_inv(a.frame_inv) {};

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  Eigen::Affine3d frame;
  Eigen::Affine3d frame_inv;

  static const Frame Identity()
  {
    return Frame(Eigen::Affine3d::Identity());
  }
};


/**@brief A TrajectoryPt is the basis for a Trajectory describing the desired path a robot should execute.
 * The desired robot motion spans both Cartesian and Joint space, and so the TrajectoryPt must have capability
 * to report on both these properties.
 *
 * In practice, an application will create a series of process points,
 * and use these process points to create a Trajectory that can be solved for a robot path.
 * In order to implement this easily, each process point should keep track of the TrajectoryPt id, and
 * provide an interpolation method between points.
 *
 */
class TrajectoryPt
{
public:
  TrajectoryPt() {};
  virtual ~TrajectoryPt() {};

  /**@name Getters for Cartesian pose(s)
   * References to "closest" position are decided by norm of joint-space distance.
   * @{
   */

  /**@brief Get single Cartesian pose associated with closest position of this point to seed_state.
   * @param pose If successful, affine pose of this state.
   * @param seed_state RobotState used for kinematic calculations and joint_position seed.
   * @return True if calculation successful. pose untouched if return false.
   */
  virtual bool getClosestCartPose(Eigen::Affine3d &pose, const moveit::core::RobotState &seed_state) const = 0;

  /**@brief Get single Cartesian pose associated with nominal of this point.
    * @param pose If successful, affine pose of this state.
    * @param seed_state RobotState used for kinematic calculations and joint_position seed.
    * @return True if calculation successful. pose untouched if return false.
    */
  virtual bool getNominalCartPose(Eigen::Affine3d &pose, const moveit::core::RobotState &seed_state) const = 0;
  /** @} (end section) */

  /**@name Getters for joint pose(s)
   * References to "closest" position are decided by norm of joint-space distance.
   * @{
   */

  /**@brief Get single Joint pose closest to seed_state.
   * @param joint_pose Solution (if function successful).
   * @param seed_state RobotState used for kinematic calculations and joint position seed.
   * @return True if calculation successful. joint_pose untouched if return is false.
   */
  virtual bool getClosestJointPose(std::vector<double> &joint_pose, const moveit::core::RobotState &seed_state) const = 0;

  /**@brief Get single Joint pose closest to seed_state.
   * @param joint_pose Solution (if function successful).
   * @param seed_state RobotState used kinematic calculations and joint position seed.
   * @return True if calculation successful. joint_pose untouched if return is false.
   */
  virtual bool getNominalJointPose(std::vector<double> &joint_pose, const moveit::core::RobotState &seed_state) const = 0;
  /** @} (end section) */

  /**@brief Check if state satisfies trajectory point requirements. */
  virtual bool isValid(const moveit::core::RobotState &state) const = 0;

  /**@name Getters/Setters for ID
   * @{ */

  /**@brief Get ID associated with this point */
  inline
  size_t getID() const
  {
    return id_;
  }

  /**@brief Set ID for this point.
   * @param id Number to set id_ to.
   */
  void setID(size_t id)
  {
    id_ = id;
  }
  /** @} (end section) */

protected:
  size_t                        id_;                    /**<@brief ID associated with this pt. Generally refers back to a process path defined elsewhere. */
  TrajectoryPtTransitionPtr     transition_;            /**<@brief Velocities at, and interpolation method to reach this point **/

};

} /* namespace descartes */
#endif /* TRAJECTORY_PT_H_ */
