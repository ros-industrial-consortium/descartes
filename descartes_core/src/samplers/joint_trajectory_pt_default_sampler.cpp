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

#include "descartes_core/samplers/joint_trajectory_pt_default_sampler.h"
#include "ros/console.h"


namespace descartes_core
{

JointTrajectoryPtDefaultSampler::SamplerState& JointTrajectoryPtDefaultSampler::SamplerState::operator ++()
{
  /* Check if sampling is done. Termination condition is that all joints provided all samples. */
  done = true;
  for (size_t joint_idx=0; joint_idx<samples.size(); ++joint_idx)
  {
    if (sample_idx[joint_idx] != samples[joint_idx].size()-1)
    {
      done = false;
      break;
    }
  }
  if (done)
  {
    return *this;
  }

  /* Walk along each joint.
   *   If all samples have been taken of that joint, reset it to 0
   *   If more samples remain, increment sample counter and stop looking
   */
  for (size_t joint_idx=0; joint_idx<samples.size(); ++joint_idx)
  {
    if (sample_idx[joint_idx] == samples[joint_idx].size()-1)
    {
      sample_idx[joint_idx] = 0;
    }
    else
    {
      ++sample_idx[joint_idx];
      break;
    }
  }

  return *this;
}

JointTrajectoryPtDefaultSampler::SamplerState JointTrajectoryPtDefaultSampler::SamplerState::operator ++(int)
{
  SamplerState tmp(*this);
  ++(*this);
  return tmp;
}

void JointTrajectoryPtDefaultSampler::SamplerState::clear()
{
  samples.clear();
  sample_idx.clear();
  done = false;
}

std::vector<double> JointTrajectoryPtDefaultSampler::SamplerState::current_sample()
{
  std::vector<double> sample(samples.size());
  for (size_t ii=0; ii<sample.size(); ++ii)
  {
    sample[ii] = samples[ii][sample_idx[ii]];
  }
  return sample;
}


inline
std::vector<double> JointTrajectoryPtDefaultSampler::getSampleIncrement() const
{
  return increment_;
}

inline
void JointTrajectoryPtDefaultSampler::setSampleIncrement(const std::vector<double> &values)
{
  increment_     = values;
}

bool JointTrajectoryPtDefaultSampler::initPositionData(const std::vector<double> &nominal, const std::vector<double> &upper_bound, const std::vector<double> &lower_bound)
{
  sampler_state_.clear();
  size_t n = nominal.size();

  if (increment_.size() != n)
  {
    ROS_ERROR("Could not initialize sampler because discretization is incorrect size.");
    return false;
  }
  if (upper_bound.size() != n || lower_bound.size() != n)
  {
    ROS_ERROR("Bounds not same size as nominal.");
    return false;
  }

  sampler_state_.samples.resize(n);
  sampler_state_.sample_idx = std::vector<size_t>(n,0);

  /* Populate samples
   * Set joint discretization <= discretization_ such that upper/lower bounds of joint are included.
   * Each joint is populated with [nom, nom+1, nom-1, nom+2, ...] until each bound is reached.
   * In this manner, each sample steps incrementally farther away from the nominal.
   */
  for (size_t ii=0; ii<n; ++ii)
  {
    size_t upper_count = increment_[ii] != 0 ? std::ceil((upper_bound[ii] - nominal[ii]) / increment_[ii] - 2.0*std::numeric_limits<double>::epsilon()) : 0,
           lower_count = increment_[ii] != 0 ? std::ceil((nominal[ii] - lower_bound[ii]) / increment_[ii] - 2.0*std::numeric_limits<double>::epsilon()) : 0; //TODO how else to get rid of rounding error?
    double upper_increment = upper_count != 0 ? (upper_bound[ii] - nominal[ii]) / (double)upper_count : std::numeric_limits<double>::max(),
           lower_increment = lower_count != 0 ? (nominal[ii] - lower_bound[ii]) / (double)lower_count : std::numeric_limits<double>::max();

    sampler_state_.samples[ii].push_back(nominal[ii]);
    for (size_t jj=0; jj<std::max(upper_count, lower_count); ++jj)
    {
      if (jj < upper_count)
      {
        sampler_state_.samples[ii].push_back(nominal[ii] + double(jj) * upper_increment);
      }
      if (jj < lower_count)
      {
        sampler_state_.samples[ii].push_back(nominal[ii] - double(jj) * lower_increment);
      }
    }
  }

  return true;
}

bool JointTrajectoryPtDefaultSampler::sample(std::vector<double> &result)
{
  if (sampler_state_.done)
  {
    return false;
  }

  std::vector<double> solution = sampler_state_.current_sample();
  if (solution.size() == 0)
  {
    return false;
  }

  ++sampler_state_;
  result = solution;
  return true;
}

} /* namespace descartes_core */
