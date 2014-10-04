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

//#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "descartes_trajectory_planning/trajectory_pt_transition.h"

namespace descartes
{

/**@brief Frame is a wrapper for an affine frame transform.
 * Frame inverse can also be stored for increased speed in downstream calculations.
 */
struct Frame
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  Eigen::Affine3d frame;
  Eigen::Affine3d frame_inv;
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
  TrajectoryPt();
  virtual ~TrajectoryPt();

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

protected:
  size_t                        id_;                    /**<@brief ID associated with this pt. Generally refers back to a process path defined elsewhere. */
  TrajectoryPtTransitionPtr     transition_;            /**<@brief Velocities at, and interpolation method to reach this point **/

};

} /* namespace descartes */
#endif /* TRAJECTORY_PT_H_ */
