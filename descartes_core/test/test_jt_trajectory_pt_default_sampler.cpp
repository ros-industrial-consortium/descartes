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
 * test_jt_trajectory_pt_default_sampler.cpp
 *
 *  Created on: Dec 7, 2014
 *      Author: Dan Solomon
 */

#include "descartes_core/samplers/joint_trajectory_pt_default_sampler.h"
#include "descartes_core/joint_trajectory_pt.h"
#include <gtest/gtest.h>

using namespace descartes_core;


TEST(JointTrajPtDefaultSampler, externallyCalledSampler1)
{
  std::vector<double> nominal(6, 0.), upper_bound(6, 0.), lower_bound(6, 0.);  /* Set nominal, upper, and lower all to 6x0 */
  descartes_core::JointTrajectoryPtDefaultSampler sampler;
  sampler.setSampleIncrement(std::vector<double>(6,1.));
  sampler.initPositionData(nominal, upper_bound, lower_bound);

  /* Test expected behavior */
  std::vector<std::vector<double> > solutions;
  std::vector<double> sample_solution;
  while(sampler.sample(sample_solution))
  {
    solutions.push_back(sample_solution);
  }
  EXPECT_EQ(solutions.size(), 1);    /* Only 1 sample should be created (because pt has 0 tolerance) */

  /* No additional samples should remain */
  EXPECT_FALSE(sampler.sample(sample_solution));
}

TEST(JointTrajPtDefaultSampler, externallyCalledSampler2)
{
  std::vector<double> nominal(2, 0.), upper_bound(2, 2.), lower_bound(2, -2.);  /* Set nominal to 2x0., upper, and lower to 2x2. */
  descartes_core::JointTrajectoryPtDefaultSampler sampler;
  sampler.setSampleIncrement(std::vector<double>(2,1.));
  sampler.initPositionData(nominal, upper_bound, lower_bound);

  /* Test expected behavior */
  std::vector<std::vector<double> > solutions;
  std::vector<double> sample_solution;
  while(sampler.sample(sample_solution))
  {
    solutions.push_back(sample_solution);
  }
  EXPECT_EQ(solutions.size(), 25);    /* Each of {n} joints has {(upper-lower)/increment+1} possibilities, ((range+1)/increment)^n => 5^2 */

  /* No additional samples should remain */
  EXPECT_FALSE(sampler.sample(sample_solution));
}

TEST(JointTrajPtDefaultSampler, internallyCalledSampler)
{
  size_t n(2);  /* DOF */
  double tolerance = 0.2;
  double increment = 0.1;
  descartes_core::JointTrajectoryPt pt;
  std::vector<TolerancedJointValue> joints(n);
  for (size_t ii=0; ii<n; ++ii)
  {
    TolerancedJointValue tjv;
    tjv.nominal = 0.1;
    tjv.tolerance.lower = tjv.tolerance.upper = tolerance;
    joints[ii] = tjv;
  }
  pt.setJoints(joints); /* n joints set to position 0.1, (+/- 0.2 tolerance) */
  std::vector<double> discretization(n, increment);
  pt.setDiscretization(discretization);    /* Note: discretization assigned to TrajectoryPt. Will be passed to sampler in setSampler() */

  JointPtSamplerBasePtr sampler(new JointTrajectoryPtDefaultSampler);
  EXPECT_TRUE(pt.setSampler(sampler));                  /* Performs sampler initialization here */

  std::vector<std::vector<double> > sols;
  sols = pt.sample(0);
  EXPECT_EQ(sols.size(), std::pow((2*tolerance)/increment+1,n));

  sols.clear();
  sols = pt.sample(5);       /* No more samples left */
  EXPECT_EQ(sols.size(), 0);

  pt.setSampler(sampler);    /* Reinitialize sampler */
  sols.clear();
  sols = pt.sample(5);       /* Get first 5 samples */
  EXPECT_EQ(sols.size(), 5);

}
