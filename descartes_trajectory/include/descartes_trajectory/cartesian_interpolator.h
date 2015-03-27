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

#ifndef CARTESIAN_INTERPOLATOR_H_
#define CARTESIAN_INTERPOLATOR_H_

#include <descartes_trajectory/cart_trajectory_pt.h>
#include <descartes_trajectory/trajectory_segment.h>
#include <limits>
#include <tf/transform_datatypes.h>

namespace descartes_trajectory
{

class CartesianInterpolator
{
public:
  CartesianInterpolator();
  virtual ~CartesianInterpolator();

  /*
   * @brief Initializes the interpolator
   * @param robot_model   A robot model implementation
   * @param tool_speed    Magnitude of the speed vector followed by the robot tool
   * @param time_step     Time elapsed between each point in the interpolated trajectory
   * @param zone_radius   Radius around each point in the coarse trajectory where parabolic
   *                      blending starts (not used at the moment)
   */
  bool initialize(descartes_core::RobotModelConstPtr robot_model, double tool_speed,double time_step,
                  double zone_radius);

  /*
   * @brief Interpolates a coarse trajectory in cartesian space
   * @param coarse_traj       The coarse trajectory that is to be interpolated
   * @param interpolated_traj The interpolated trajectory
   */
  bool interpolate(const std::vector<descartes_core::TrajectoryPtPtr>& coarse_traj,
                   std::vector<descartes_core::TrajectoryPtPtr>& interpolated_traj);
protected:

  bool computeFullSegmentTimes(const std::vector<descartes_core::TrajectoryPtPtr>& coarse_traj,
                               std::vector<double>& ft);

protected:

  descartes_core::RobotModelConstPtr robot_model_;
  double tool_speed_; // magnitude of the tool's linear displacement
  double time_step_ ; // time step between interpolated points
  double zone_radius_; // radius around each point where parabolic blending starts
};

} /* namespace descartes_trajectory */
#endif /* CARTESIAN_INTERPOLATOR_H_ */
