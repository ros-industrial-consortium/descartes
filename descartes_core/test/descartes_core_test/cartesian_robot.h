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

#ifndef CARTESIAN_ROBOT_H_
#define CARTESIAN_ROBOT_H_

#include "descartes_core/robot_model.h"

namespace descartes_core_test
{

/**@brief Cartesian Robot used for test purposes.  This is a simple robot with simple kinematics.  Each
 * joint corresponds to a cartesian direction (i.e. x, y, R, P, Y) (don't ask me how this is built, it
 * just works).
*/
class CartesianRobot : public descartes_core::RobotModel
{
public:

  CartesianRobot() : pos_limit_(1.0), orient_limit_(M_PI)
  {
  }
  ;
  CartesianRobot(double pos_limit, double orient_limit) : pos_limit_(pos_limit), orient_limit_(orient_limit)
  {
  }
  ;

  virtual bool getIK(const Eigen::Affine3d &pose, const std::vector<double> &seed_state,
                     std::vector<double> &joint_pose) const;

  virtual bool getAllIK(const Eigen::Affine3d &pose, std::vector<std::vector<double> > &joint_poses) const;

  virtual bool getFK(const std::vector<double> &joint_pose, Eigen::Affine3d &pose) const;

  virtual bool isValid(const std::vector<double> &joint_pose) const;

  virtual bool isValid(const Eigen::Affine3d &pose) const;

  double pos_limit_;
  double orient_limit_;

};


}

#endif // CARTESIAN_ROBOT_H
