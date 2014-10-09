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
/*
 * joint_trajectory_pt.h
 *
 *  Created on: Oct 3, 2014
 *      Author: Dan Solomon
 */

#ifndef JOINT_TRAJECTORY_PT_H_
#define JOINT_TRAJECTORY_PT_H_

#include <vector>
#include "descartes_trajectory_planning/trajectory_pt.h"

namespace descartes
{

//TODO add warning if non-zero tolerances are specified because initial implementation will only allow fixed joints
struct TolerancedJointValue
{
  TolerancedJointValue():tol_above(0.), tol_below(0.) {};
  TolerancedJointValue(double _nominal, double _tol_above, double _tol_below):
    nominal(_nominal), tol_above(_tol_above), tol_below(_tol_below) {};
  TolerancedJointValue(double _nominal)
  {
    *this = TolerancedJointValue(_nominal, 0., 0.);
  }

  double upperBound() const
  {
    return nominal+tol_above;
  }

  double lowerBound() const
  {
    return nominal-tol_below;
  }

  double range() const
  {
    return upperBound() - lowerBound();
  }

  double nominal;
  double tol_above, tol_below;
};

/**@brief Joint Trajectory Point used to describe a joint goal for a robot trajectory.
 * (see TrajectoryPt class documentation for background on terms).
 * For a JointTrajectoryPt, the transform from wrist to tool, and base to object, are defined by fixed frames.
 * These transforms are important when calculating interpolation.
 * The joint position is specified as a nominal with upper/lower bounds.
 */
class JointTrajectoryPt: public TrajectoryPt
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;      //TODO is this needed when Frame already has it?
public:
  JointTrajectoryPt();
  virtual ~JointTrajectoryPt() {};

  /**@name Getters for Cartesian pose(s)
   * @{
   */

  //TODO complete
  /**@brief Get single Cartesian pose associated with closest position of this point to seed_state.
   * (Pose of TOOL point expressed in WORLD frame).
   * @param pose If successful, affine pose of this state.
   * @param seed_state RobotState used for kinematic calculations and joint_position seed.
   * @return True if calculation successful. pose untouched if return false.
   */
  virtual bool getClosestCartPose(Eigen::Affine3d &pose, const moveit::core::RobotState &seed_state) const;

  //TODO complete
  /**@brief Get single Cartesian pose associated with nominal of this point.
    * (Pose of TOOL point expressed in WORLD frame).
    * @param pose If successful, affine pose of this state.
    * @param seed_state RobotState used for kinematic calculations and joint_position seed.
    * @return True if calculation successful. pose untouched if return false.
    */
  virtual bool getNominalCartPose(Eigen::Affine3d &pose, const moveit::core::RobotState &seed_state) const;

  //TODO complete
  /**@brief Get "all" Cartesian poses that satisfy this point. Use RobotState for performing kinematic calculations.
   * @param poses Note: Number of poses returned may be subject to discretization used.
   * @param state RobotState used for kinematic calculations.
   */
  virtual void getCartesianPoses(EigenSTL::vector_Affine3d &poses, const moveit::core::RobotState &state) const;
  /** @} (end section) */

  /**@name Getters for joint pose(s)
   * @{
   */

  //TODO complete
  /**@brief Get single Joint pose closest to seed_state.
   * @param joint_pose Solution (if function successful).
   * @param seed_state RobotState used for kinematic calculations and joint position seed.
   * @return True if calculation successful. joint_pose untouched if return is false.
   */
  virtual bool getClosestJointPose(std::vector<double> &joint_pose, const moveit::core::RobotState &seed_state) const;

  //TODO complete
  /**@brief Get single Joint pose closest to seed_state.
   * @param joint_pose Solution (if function successful).
   * @param seed_state RobotState used kinematic calculations and joint position seed.
   * @return True if calculation successful. joint_pose untouched if return is false.
   */
  virtual bool getNominalJointPose(std::vector<double> &joint_pose, const moveit::core::RobotState &seed_state) const;

  //TODO complete
  /**@brief Get "all" joint poses that satisfy this point.
   * @param joint_poses vector of solutions (if function successful). Note: # of solutions may be subject to discretization used.
   * @param seed_state RobotState used for kinematic calculations.
   */
  virtual void getJointPoses(std::vector<std::vector<double> > &joint_poses, const moveit::core::RobotState &state) const;
  /** @} (end section) */

  //TODO complete
  /**@brief Check if state satisfies trajectory point requirements. */
  virtual bool isValid(const moveit::core::RobotState &state) const;

  //TODO complete
  /**@brief Set discretization. Each joint can have a different discretization.
   * @param discretization Vector of discretization values. If length=1, set all elements of discretization_ are set to value.
   * @return True if vector is length 1 or length(joint_position_) and value[ii] are within 0-range(joint_position[ii]).
   */
  virtual bool setDiscretization(const std::vector<double> &discretization);


protected:
  std::vector<TolerancedJointValue> joint_position_;  /**<@brief Fixed joint position with tolerance */
  std::vector<double>               discretization_;  /**<@brief How finely to discretize each joint */

  /** @name JointTrajectoryPt transforms. Used in get*CartPose() methods and for interpolation.
   *  @{
   */
  Frame                         tool_;                  /**<@brief Transform from robot wrist to active tool pt. */
  Frame                         wobj_;                  /**<@brief Transform from world to active workobject pt. */
  /** @} (end section) */

};

} /* namespace descartes */

#endif /* JOINT_TRAJECTORY_PT_H_ */
