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

#include <descartes_core/samplers/joint_trajectory_pt_default_sampler.h>
#include <boost/uuid/uuid_io.hpp>


namespace descartes_core
{

JointTrajectoryPtDefaultSampler::~JointTrajectoryPtDefaultSampler()
{}

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

std::vector<double> JointTrajectoryPtDefaultSampler::SamplerState::current_sample()
{
  JointSample sample(samples.size());
  for (size_t ii=0; ii<sample.size(); ++ii)
  {
    sample[ii] = samples[ii][sample_idx[ii]];
  }
  return sample;
}

bool JointTrajectoryPtDefaultSampler::init(const TrajectoryPt &pt)
{
  const JointTrajectoryPt::JointPointData* joints = static_cast<const JointTrajectoryPt::JointPointData*>(pt.getPointData());
  //TODO this has to return false if cast doesn't work. Somehow incorporate dynamic cast?

  sampler_state_.clear();
  size_t n = joints->joint_position.size();
  sampler_state_.samples.resize(n);
  sampler_state_.sample_idx = std::vector<size_t>(n,0);

  /* Populate samples
   * Joint discretization set<=discretization_ such that upper/lower bounds of joint are included.
   * Each joint is populated with [nom, nom+1, nom-1, nom+2, ...] until each bound is reached.
   * In this manner, each sample steps incrementally farther away from the nominal.
   */
  for (size_t ii=0; ii<n; ++ii)
  {
    double nom = joints->joint_position.at(ii).nominal,
           upper = joints->joint_position.at(ii).upperBound(),
           lower = joints->joint_position.at(ii).lowerBound();
    size_t upper_count = std::ceil((upper - nom) / discretization_[ii] - 2.0*std::numeric_limits<double>::epsilon()),
           lower_count = std::ceil((nom - lower) / discretization_[ii] - 2.0*std::numeric_limits<double>::epsilon()); //TODO how else to get rid of rounding error?
//    std::cout << "Upper/lower count " << upper_count << "/" << lower_count << std::endl;
//    std::cout << nom << "/" << upper << "/" << lower << std::endl;
    double upper_increment = upper_count != 0 ? (upper - nom)/(double)upper_count : std::numeric_limits<double>::max(),
           lower_increment = lower_count != 0 ? (nom - lower)/(double)lower_count : std::numeric_limits<double>::max();

    sampler_state_.samples[ii].push_back(nom);
    for (size_t jj=0; jj<std::max(upper_count, lower_count); ++jj)
    {
      if (jj < upper_count)
      {
        sampler_state_.samples[ii].push_back(nom + double(jj) * upper_increment);
      }
      if (jj < lower_count)
      {
        sampler_state_.samples[ii].push_back(nom - double(jj) * lower_increment);
      }
    }
  }

  return true;
}

const void* JointTrajectoryPtDefaultSampler::sample(size_t n, const TrajectoryPt &pt)
{
  solution_storage_.clear();
  bool get_all(n==0);
  if (get_all)
  {
    n = 1;
  }

  size_t ii(0);
  while (!sampler_state_.done && ii<n)
  {
    std::vector<double> solution = sampler_state_.current_sample();
    ++sampler_state_;
    if (solution.size() > 0)
    {
      solution_storage_.push_back(solution);
    }

    if (!get_all)
    {
      ++ii;
    }
  }

  return (const void*) &solution_storage_;
}

} /* namespace descartes_core */
