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
 * trajectory_segment.h
 *
 *  Created on: Mar 26, 2015
 *      Author: Jorge Nicho
 */

#ifndef DESCARTES_TRAJECTORY_TRAJECTORY_SEGMENT_H_
#define DESCARTES_TRAJECTORY_TRAJECTORY_SEGMENT_H_

#include <descartes_trajectory/cart_trajectory_pt.h>
#include <tf/transform_datatypes.h>

namespace descartes_trajectory
{

class TrajectorySegment
{
public:

  virtual ~TrajectorySegment()
  {

  }

  /*
   * @brief Interface method that should generate intermediate points in accordance to the supported interpolation strategy
   * @param segment_points      Array to be populated with the interpolated points and includes the start and end points
   */
  virtual bool interpolate(std::vector<descartes_core::TrajectoryPtPtr>& segment_points) = 0;

protected:

  TrajectorySegment(){}

};

class BlendSegment: public TrajectorySegment
{
public:

  /*
   * brief Generates an interpolated blend region at a constant acceleration of the tool. Orientation is interpolated
   * linearly with time.
   * @param start_pose  The pose where the blend region starts
   * @param end_pose    The pose where the blend region ends
   * @param start_vel   The tool velocity at the start of the blend region
   * @param end_vel     The tool velocity at the end of the blend region
   * @param time_step   Time elapsed between each point in the interpolated segment.  The total number of points will equal
   *                    (blend segment duration/time_step) + 1.  The blend segment duration is computed using
   *                    the position and speeds at the start and end points.
   */
  BlendSegment(const Eigen::Affine3d& start_pose,
               const Eigen::Affine3d& end_pose,
               const Eigen::Vector3d& start_vel,
               const Eigen::Vector3d& end_vel,
               double time_step);
  virtual ~BlendSegment();

  virtual bool interpolate(std::vector<descartes_core::TrajectoryPtPtr>& segment_points);

protected:

  Eigen::Affine3d start_pose_;
  Eigen::Affine3d end_pose_;
  Eigen::Vector3d start_vel_;
  Eigen::Vector3d end_vel_;
  Eigen::Vector3d accel_;
  double duration_;
  double time_step_;

};

class LinearSegment: public TrajectorySegment
{
public:

  /*
   * @brief Interpolates between two points at a constant speed of the tool. Orientation is interpolated
   * linearly with time.
   * @param start_pose  The pose at the start of the segment
   * @param end_pose    The pose at the end of the segment
   * @param duration    The total time duration of the segment
   * @param time_step   Time elapsed between each point in the interpolated segment.  The total number of points will equal
   *                    (duration/time_step) + 1.
   * @param interval    A pair that contains the time interval where points will be generated. It should be bounded
   *                    by t>0 and  t < duration.  It defaults to <0,duration>
   *
   */
  LinearSegment(const Eigen::Affine3d& start_pose,
                const Eigen::Affine3d& end_pose,
                double duration,
                double time_step,
                const std::pair<double,double>& interval = std::make_pair(0,std::numeric_limits<double>::max()));

  virtual ~LinearSegment();

  virtual bool interpolate(std::vector<descartes_core::TrajectoryPtPtr>& segment_points);

protected:

  Eigen::Affine3d start_pose_;
  Eigen::Affine3d end_pose_;
  double duration_;
  double time_step_;
  std::pair<double,double> interval_;

};

} /* namespace descartes_trajectory */

#endif /* DESCARTES_TRAJECTORY_TRAJECTORY_SEGMENT_H_ */
