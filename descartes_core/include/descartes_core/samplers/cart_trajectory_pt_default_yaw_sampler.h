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

#ifndef CART_TRAJECTORY_PT_DEFAULT_YAW_SAMPLER_H_
#define CART_TRAJECTORY_PT_DEFAULT_YAW_SAMPLER_H_

#include <descartes_core/samplers/cart_pt_sampler_base.h>


namespace descartes_core
{

class CartTrajectoryPtDefaultYawSampler : public CartPtSamplerBase
{
public:
  virtual ~CartTrajectoryPtDefaultYawSampler() {};

  virtual
  std::vector<double> getSampleIncrement() const;
  virtual
  void setSampleIncrement(const std::vector<double> &values);

  virtual
  bool initPositionData(const Eigen::Affine3d &nominal,
                        const Eigen::Vector3d &upper_position_bound = Eigen::Vector3d::Zero(), const Eigen::Vector3d &lower_position_bound = Eigen::Vector3d::Zero(),
                        const Eigen::Vector3d &upper_rpy_bound = Eigen::Vector3d::Zero(), const Eigen::Vector3d &lower_rpy_bound = Eigen::Vector3d::Zero());

  virtual
  bool sample(Eigen::Affine3d &result);

protected:

  /* Private Members */
  std::vector<double> increment_;
  Eigen::Affine3d nominal_;
  std::vector<double> yaw_samples_;
  size_t sample_idx_;

};

} /* namespace descartes_core */

#endif /* CART_TRAJECTORY_PT_DEFAULT_YAW_SAMPLER_H_ */
