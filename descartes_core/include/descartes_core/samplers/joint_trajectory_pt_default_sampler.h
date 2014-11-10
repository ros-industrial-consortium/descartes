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
 * trajectory_pt_sampler.h
 *
 *  Created on: Oct 29, 2014
 *      Author: Dan Solomon
 */

#ifndef JOINT_TRAJECTORY_PT_DEFAULT_SAMPLER_H_
#define JOINT_TRAJECTORY_PT_DEFAULT_SAMPLER_H_

#include <descartes_core/samplers/trajectory_pt_sampler.h>
#include <descartes_core/joint_trajectory_pt.h>


namespace descartes_core
{

class JointTrajectoryPtDefaultSampler : public TrajectoryPtSampler
{
public:
  typedef std::vector<std::vector<double> > SampleData;

public:
  virtual ~JointTrajectoryPtDefaultSampler();

  inline virtual
  std::vector<double> getDiscretization() const
  {
    return discretization_;
  }
  inline virtual
  void setDiscretization(const std::vector<double> &values)
  {
    discretization_ = values;
  }

  virtual
  bool init(const TrajectoryPt &pt);

  virtual
  const void* sample(size_t n, const TrajectoryPt &pt);

protected:

  struct SamplerState
  {
    typedef std::vector<double> JointSample;
    std::vector<JointSample> samples;
    std::vector<size_t> sample_idx;
    bool done;

    JointSample current_sample();

    void clear()
    {
      samples.clear();
      sample_idx.clear();
      done = false;
    }

    /* prefix */
    SamplerState& operator++ ();

    /* postfix */
    SamplerState operator++ (int)
    {
      SamplerState tmp(*this);
      ++(*this);
      return tmp;
    }
  };

  /* Private Members */
  std::vector<double> discretization_;
  SamplerState sampler_state_;

private:

  SampleData solution_storage_;  /**<@brief storage for solutions created by sample() */

};

} /* namespace descartes_core */

#endif /* JOINT_TRAJECTORY_PT_DEFAULT_SAMPLER_H_ */
