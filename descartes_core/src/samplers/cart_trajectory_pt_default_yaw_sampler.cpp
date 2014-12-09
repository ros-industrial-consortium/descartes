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
 * cart_trajectory_pt_default_yaw_sampler.h
 *
 *  Created on: Dec 9, 2014
 *      Author: Dan Solomon
 */

#include "descartes_core/samplers/cart_trajectory_pt_default_yaw_sampler.h"
#include "ros/console.h"


namespace descartes_core
{

inline
std::vector<double> CartTrajectoryPtDefaultYawSampler::getSampleIncrement() const
{
  return increment_;
}

inline
void CartTrajectoryPtDefaultYawSampler::setSampleIncrement(const std::vector<double> &values)
{
  increment_ = values;
}

bool CartTrajectoryPtDefaultYawSampler::initPositionData(const Eigen::Affine3d &nominal,
                                                         const Eigen::Vector3d &upper_position_bound, const Eigen::Vector3d &lower_position_bound,
                                                         const Eigen::Vector3d &upper_rpy_bound, const Eigen::Vector3d &lower_rpy_bound)
{
  if (increment_.size() != 6)
  {
    ROS_ERROR("Could not initialize sampler because discretization (%lu) is not correct size (6).", increment_.size());
    return false;
  }

  /* Populate samples
   * Each yaw sample is populated with [nom, nom+1, nom-1, nom+2, ...] until each bound is reached.
   * In this manner, each sample steps incrementally farther away from the nominal.
   */
  Eigen::Vector3d euler = nominal.rotation().eulerAngles(2, 1, 0);
  double nominal_yaw = euler(0);
  const double& upper_yaw_bound = upper_rpy_bound(2),
                lower_yaw_bound = lower_rpy_bound(2),
                yaw_discretization = increment_[5];
  size_t upper_count = yaw_discretization != 0 ? std::ceil((upper_yaw_bound - nominal_yaw) / yaw_discretization - 2.0*std::numeric_limits<double>::epsilon()) : 0,
         lower_count = yaw_discretization != 0 ? std::ceil((nominal_yaw - lower_yaw_bound) / yaw_discretization - 2.0*std::numeric_limits<double>::epsilon()) : 0; //TODO how else to get rid of rounding error?
  double upper_increment = upper_count != 0 ? (upper_yaw_bound - nominal_yaw) / (double)upper_count : std::numeric_limits<double>::max(),
         lower_increment = lower_count != 0 ? (nominal_yaw - lower_yaw_bound) / (double)lower_count : std::numeric_limits<double>::max();

  yaw_samples_.push_back(nominal_yaw);
  for (size_t ii=0; ii<std::max(upper_count, lower_count); ++ii)
  {
    if (ii < upper_count)
    {
      yaw_samples_.push_back(nominal_yaw + double(ii) * upper_increment);
    }
    if (ii < lower_count)
    {
      yaw_samples_.push_back(nominal_yaw - double(ii) * lower_increment);
    }
  }

  sample_idx_ = 0;

  return true;
}

bool CartTrajectoryPtDefaultYawSampler::sample(Eigen::Affine3d &result)
{
  if (sample_idx_ == yaw_samples_.size())
  {
    return false;
  }

  result = nominal_ * Eigen::AngleAxisd(yaw_samples_[sample_idx_], Eigen::Vector3d::UnitZ());  //TODO check that this is correct yaw axis. Better yet, set yaw axis somewhere.
  ++sample_idx_;

  return true;
}

} /* namespace descartes_core */
