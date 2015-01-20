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

#ifndef ROBOT_KINEMATICS_H_
#define ROBOT_KINEMATICS_H_

//TODO: The include below picks up Eigen::Affine3d, but there is probably a better way
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include "descartes_core/utils.h"

namespace descartes_core
{

DESCARTES_CLASS_FORWARD(RobotModel);

/**@brief RobotModel defines the interface to a kinematics/dynamics functions.  Implementations
 * of this class will be used in conjunction with TrajectoryPt objects to determine forward
 * and inverse kinematics
 *
 * All methods in this interface class assume a *FIXED* TOOL & WOBJ frame (see TrajectoryPt
 * for frame definitions).  The methods for setting/getting these frames are not defined by
 * this class.  Implementations of this interface should provide these either by construction
 * or getter/setter methods.
 */
class RobotModel
{
public:

  virtual ~RobotModel(){}

  /**
   * @brief Returns the joint pose closest to the seed pose for a desired affine pose
   * @param pose Affine pose of TOOL in WOBJ frame
   * @param seed_state Joint position seed (returned solution is "close" to the seed).
   * @param joint_pose Solution (if function successful).
   * @return True if successful
   */
  virtual bool getIK(const Eigen::Affine3d &pose, const std::vector<double> &seed_state,
                     std::vector<double> &joint_pose) const = 0;

  /**
   * @brief Returns "all" the joint poses("distributed" in joint space) for a desired affine pose.
   * "All" is determined by each implementation (In the worst case, this means at least getIK).
   * "Distributed" is determined by each implementation.
   * @param pose Affine pose of TOOL in WOBJ frame
   * @param joint_poses Solution (if function successful).
   * @return True if successful
   */
  virtual bool getAllIK(const Eigen::Affine3d &pose, std::vector<std::vector<double> > &joint_poses) const = 0;

  /**
   * @brief Returns the affine pose
   * @param joint_pose Solution (if function successful).
   * @param pose Affine pose of TOOL in WOBJ frame
   * @return True if successful
   */
  virtual bool getFK(const std::vector<double> &joint_pose, Eigen::Affine3d &pose) const = 0;

  /**
   * @brief Returns number of DOFs
   * @return Int
   */
  virtual int getDOF() const = 0;

  /**
   * @brief Performs all necessary checks to determine joint pose is valid
   * @param joint_pose Pose to check
   * @return True if valid
   */
  virtual bool isValid(const std::vector<double> &joint_pose) const = 0;

  /**
   * @brief Performs all necessary checks to determine affine pose is valid
   * @param pose Affine pose of TOOL in WOBJ frame
   * @return True if valid
   */
  virtual bool isValid(const Eigen::Affine3d &pose) const = 0;

  virtual void initialize(const std::string robot_description, const std::string& group_name,
                          const std::string& world_frame,const std::string& tcp_frame) = 0;

protected:

  RobotModel(){}

};

}//descartes_core




#endif /* ROBOT_KINEMATICS_H_ */
