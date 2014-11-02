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
 * test_joint_trajectory_pt_default_sampler.cpp
 *
 *  Created on: Nov 1, 2014
 *      Author: dpsolomon
 */

#include "descartes_core/samplers/joint_trajectory_pt_default_sampler.h"
#include "descartes_core/trajectory_pt.h"
#include <gtest/gtest.h>

using namespace descartes_core;
typedef descartes_core::JointTrajectoryPtDefaultSampler::SampleData SampleData;


size_t factorial(size_t n)
{
  size_t res(1);
  for (size_t ii=2; ii<=n; ++ii)
  {
    res *= ii;
  }
  return res;
}

TEST(JointTrajPtDefaultSampler, externallyCalledSamplerSimple)
{

  descartes_core::JointTrajectoryPt pt(std::vector<double>(6, 0.));  /* Default JointPt has 0 tolerance */
  descartes_core::JointTrajectoryPtDefaultSampler sampler;
  sampler.setDiscretization(std::vector<double>(6,1.));
  sampler.init(pt);

  /* Test expected behavior */
  const SampleData* solutions = static_cast<const SampleData* >(sampler.sample(0, pt));   /* Get all samples */
  EXPECT_EQ(solutions->size(), 1);    /* Only 1 sample should be created (because pt has 0 tolerance) */

  /* No additional samples should remain */
  solutions = static_cast<const SampleData* >(sampler.sample(0, pt));
  EXPECT_EQ(solutions->size(), 0);

}

TEST(JointTrajPtDefaultSampler, externallyCalledSamplerTwoPts)
{
  size_t n(2);  /* DOF */
  descartes_core::JointTrajectoryPt pt1(std::vector<double>(n, 0.5)),   /* n joints set to position 0.5 (0 tolerance)*/
                                    pt2;
  std::vector<TolerancedJointValue> pt2_joints(n);
  for (size_t ii=0; ii<n; ++ii)
  {
    TolerancedJointValue tjv;
    tjv.nominal = 0.1;
    tjv.tolerance.lower = tjv.tolerance.upper = 0.2;
    pt2_joints[ii] = tjv;
  }
  pt2.setJoints(pt2_joints);                                            /* n joints set to position 0.1, (+/- 0.2 tolerance) */

  JointTrajectoryPtDefaultSampler sampler1;
  sampler1.setDiscretization(std::vector<double>(n, 0.1));
  sampler1.init(pt1);

  JointTrajectoryPtDefaultSampler sampler2 = sampler1;
  sampler2.init(pt2);

  const SampleData* sols;
  sols = static_cast<const SampleData* >(sampler1.sample(0, pt1));
  EXPECT_EQ(sols->size(), 1);
  sols = static_cast<const SampleData* >(sampler2.sample(0, pt2));
  EXPECT_EQ(sols->size(), 25);

  sols = static_cast<const SampleData* >(sampler2.sample(5, pt2));       /* No more samples left */
  EXPECT_EQ(sols->size(), 0);

  sampler2.init(pt2);                   /* Reinitialize sampler for pt2 */
  sols = static_cast<const SampleData* >(sampler2.sample(5, pt2));       /* Get first 5 samples */
  EXPECT_EQ(sols->size(), 5);

  EXPECT_EQ(1, factorial(1));
  EXPECT_EQ(2, factorial(2));
  EXPECT_EQ(6, factorial(3));
  EXPECT_EQ(24, factorial(4));
  EXPECT_EQ(120, factorial(5));
  EXPECT_EQ(720, factorial(6));
}
