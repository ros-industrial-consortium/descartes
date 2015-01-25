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
 * joint_trajectory_pt_default_sampler.h
 *
 *  Created on: Oct 29, 2014
 *      Author: Dan Solomon
 */

#ifndef JOINT_TRAJECTORY_PT_DEFAULT_SAMPLER_H_
#define JOINT_TRAJECTORY_PT_DEFAULT_SAMPLER_H_

#include <descartes_core/samplers/joint_pt_sampler_base.h>


namespace descartes_core
{

class JointTrajectoryPtDefaultSampler : public JointPtSamplerBase
{
public:
  virtual ~JointTrajectoryPtDefaultSampler() {};

  virtual
  std::vector<double> getSampleIncrement() const;
  virtual
  void setSampleIncrement(const std::vector<double> &values);

  virtual
  bool initPositionData(const std::vector<double> &nominal, const std::vector<double> &upper_bound, const std::vector<double> &lower_bound);

  virtual
  bool sample(std::vector<double> &result);

protected:

  /**@brief SamplerState holds sample data and current state for a JointTrajectoryPt sampler.
   *
   * sampler.init**() initializes the SamplerState, and calls to sampler.sample() operate on it.
   */
  struct SamplerState
  {
    std::vector<std::vector<double> > samples;  /**<@brief Each joint is assigned a set of samples */
    std::vector<size_t> sample_idx;             /**<@brief An index (into @e samples) of the current sample to be taken for each joint. */
    bool done;                                  /**<@brief Flag set when all samples have been taken. */

    /**@brief Retrieve current sample by getting each sample value by corresponding sample index.
     * @return Joint position (empty vector if @e done is set.
     */
    std::vector<double> current_sample();

    /**@brief Reset the SampleState. */
    void clear();

    /**@brief prefix incrementor
     * Increments appropriate @e sample_idx. If @e done is set, does nothing.
     */
    SamplerState& operator++ ();

    /**@brief postfix incrementor (see prefix incrementor) */
    SamplerState operator++ (int);
  };

  /* Private Members */
  SamplerState sampler_state_;          /**<@brief Sampler data and current state. */
  std::vector<double> increment_;       /**<@brief Desired sample increment for each joint. */

};

} /* namespace descartes_core */

#endif /* JOINT_TRAJECTORY_PT_DEFAULT_SAMPLER_H_ */
