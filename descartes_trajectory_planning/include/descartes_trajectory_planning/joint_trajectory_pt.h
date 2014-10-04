/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, ROS-Industrial Consortium
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
 *      Author: dpsolomon
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
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
public:
  JointTrajectoryPt();
  virtual ~JointTrajectoryPt();

protected:
  std::vector<TolerancedJointValue> joint_position_;  /* Fixed joint position with tolerance */
  Frame tool_;
  Frame object_;

};

} /* namespace descartes */

#endif /* JOINT_TRAJECTORY_PT_H_ */
