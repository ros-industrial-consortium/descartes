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

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  Eigen::Affine3d frame;
  Eigen::Affine3d frame_inv;

  static const Frame Identity()
  {
    return Frame(Eigen::Affine3d::Identity());
  }
};


/**@brief A TrajectoryPt describes how a TOOL may interact with a PART to perform an automated trajectory.
 * The TOOL is something held by the robot. It is located relative to robot wrist/tool plate.
 * The PART is something that exists in the world/global environment that is not held by robot.
 * Particular descriptions of TOOL and PART are left to the derived classes.
 * Each point can also contain information relating linear/rotational velocity and movement interpolation method in transition_.
 */
class TrajectoryPt
{
public:
  TrajectoryPt() {};
  virtual ~TrajectoryPt() {};

  /**@name Getters for Cartesian pose(s)
   * @{
   */

  /**@brief Get single Cartesian pose associated with closest position of this point to seed_state.
   * (Pose of TOOL point expressed in WORLD frame).
   * @param pose If successful, affine pose of this state.
   * @param seed_state RobotState used for kinematic calculations and joint_position seed.
   * @return True if calculation successful. pose untouched if return false.
   */
  virtual bool getClosestCartPose(Eigen::Affine3d &pose, const moveit::core::RobotState &seed_state) const = 0;

  /**@brief Get single Cartesian pose associated with nominal of this point.
    * (Pose of TOOL point expressed in WORLD frame).
    * @param pose If successful, affine pose of this state.
    * @param seed_state RobotState used for kinematic calculations and joint_position seed.
    * @return True if calculation successful. pose untouched if return false.
    */
  virtual bool getNominalCartPose(Eigen::Affine3d &pose, const moveit::core::RobotState &seed_state) const = 0;

  /**@brief Get "all" Cartesian poses that satisfy this point. Use RobotState for performing kinematic calculations.
   * @param poses Note: Number of poses returned may be subject to discretization used.
   * @param state RobotState used for kinematic calculations.
   */
  virtual void getCartesianPoses(EigenSTL::vector_Affine3d &poses, const moveit::core::RobotState &state) const = 0;
  /** @} (end section) */

  /**@name Getters for joint pose(s)
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

  /**@brief Get "all" joint poses that satisfy this point.
   * @param joint_poses vector of solutions (if function successful). Note: # of solutions may be subject to discretization used.
   * @param seed_state RobotState used for kinematic calculations.
   */
  virtual void getJointPoses(std::vector<std::vector<double> > &joint_poses, const moveit::core::RobotState &state) const = 0;
  /** @} (end section) */

  /**@brief Check if state satisfies trajectory point requirements. */
  virtual bool isValid(const moveit::core::RobotState &state) const = 0;

  /**@brief Set discretization. Derived classes interpret and use discretization differently.
   * @param discretization Vector of discretization values.
   * @return True if vector is valid length/values.
   */
  virtual bool setDiscretization(const std::vector<double> &discretization) = 0;

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
