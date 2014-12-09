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
 * test_cart_trajectory_pt_default_yaw_sampler.cpp
 *
 *  Created on: Dec 9, 2014
 *      Author: Dan Solomon
 */

#include "descartes_core/samplers/cart_trajectory_pt_default_yaw_sampler.h"
#include "descartes_core/cart_trajectory_pt.h"
#include <gtest/gtest.h>

using namespace descartes_core;


size_t factorial(size_t n)
{
  size_t res(1);
  for (size_t ii=2; ii<=n; ++ii)
  {
    res *= ii;
  }
  return res;
}

TEST(CartTrajPtDefaultYawSampler, externallyCalledSampler1)
{
  /* Nominal is Identity transform, no positional variance, +- 1 radian yaw variance. */
  Eigen::Affine3d nominal(Eigen::Affine3d::Identity());
  Eigen::Vector3d upper_position_bound(Eigen::Vector3d::Zero()), lower_position_bound(Eigen::Vector3d::Zero());
  Eigen::Vector3d upper_rpy_bound(0., 0., 1.), lower_rpy_bound(0., 0., -1.);

  /* Discretization is [x,y,z,r,p,y]
   * Note: increment set to 0 position, 0 roll/pitch/yaw,
   * so no sampling beyond nominal value will take place.
   */
  std::vector<double> increment(6,0);

  /* Create sampler, initialize increment and position data */
  descartes_core::CartTrajectoryPtDefaultYawSampler sampler;
  sampler.setSampleIncrement(increment);
  sampler.initPositionData(nominal, upper_position_bound, lower_position_bound, upper_rpy_bound, lower_rpy_bound);

  /* Test expected behavior */
  EigenSTL::vector_Affine3d solutions;
  Eigen::Affine3d sample_solution;
  while(sampler.sample(sample_solution))
  {
    solutions.push_back(sample_solution);
  }
  EXPECT_EQ(solutions.size(), 1);    /* Only 1 sample should be created (because pt has 0 sample increment) */

  /* No additional samples should remain */
  EXPECT_FALSE(sampler.sample(sample_solution));
}

TEST(CartTrajPtDefaultYawSampler, externallyCalledSampler2)
{
  /* Nominal is Identity transform, no positional variance, +- 1 radian yaw variance. */
  Eigen::Affine3d nominal(Eigen::Affine3d::Identity());
  Eigen::Vector3d upper_position_bound(Eigen::Vector3d::Zero()), lower_position_bound(Eigen::Vector3d::Zero());
  Eigen::Vector3d upper_rpy_bound(0., 0., 1.), lower_rpy_bound(0., 0., -1.);

  /* Discretization is [x,y,z,r,p,y]
   * Note: increment set to 0 position, 0 roll/pitch, .1 yaw
   */
  std::vector<double> increment(6,0);
  increment[5] = 0.1;

  /* Create sampler, initialize increment and position data */
  descartes_core::CartTrajectoryPtDefaultYawSampler sampler;
  sampler.setSampleIncrement(increment);
  sampler.initPositionData(nominal, upper_position_bound, lower_position_bound, upper_rpy_bound, lower_rpy_bound);

  /* Test expected behavior */
  EigenSTL::vector_Affine3d solutions;
  Eigen::Affine3d sample_solution;
  while(sampler.sample(sample_solution))
  {
    solutions.push_back(sample_solution);
  }
  EXPECT_EQ(solutions.size(), 21);

  /* No additional samples should remain */
  EXPECT_FALSE(sampler.sample(sample_solution));
}

//TEST(CartTrajPtDefaultYawSampler, internallyCalledSampler)
//{
//  size_t n(2);  /* DOF */
//  descartes_core::JointTrajectoryPt pt;
//  std::vector<TolerancedJointValue> joints(n);
//  for (size_t ii=0; ii<n; ++ii)
//  {
//    TolerancedJointValue tjv;
//    tjv.nominal = 0.1;
//    tjv.tolerance.lower = tjv.tolerance.upper = 0.2;
//    joints[ii] = tjv;
//  }
//  pt.setJoints(joints); /* n joints set to position 0.1, (+/- 0.2 tolerance) */
//  std::vector<double> discretization(n, 0.1);
//  pt.setDiscretization(discretization);    /* Note: discretization assigned to TrajectoryPt. Will be passed to sampler in setSampler() */
//
//  JointPtSamplerBasePtr sampler(new JointTrajectoryPtDefaultSampler);
//  EXPECT_TRUE(pt.setSampler(sampler));                  /* Performs sampler initialization here */
//
//  std::vector<std::vector<double> > sols;
//  sols = pt.sample(0);
//  EXPECT_EQ(sols.size(), 25);
//
//  sols.clear();
//  sols = pt.sample(5);       /* No more samples left */
//  EXPECT_EQ(sols.size(), 0);
//
//  pt.setSampler(sampler);    /* Reinitialize sampler */
//  sols.clear();
//  sols = pt.sample(5);       /* Get first 5 samples */
//  EXPECT_EQ(sols.size(), 5);
//
//  EXPECT_EQ(1, factorial(1));
//  EXPECT_EQ(2, factorial(2));
//  EXPECT_EQ(6, factorial(3));
//  EXPECT_EQ(24, factorial(4));
//  EXPECT_EQ(120, factorial(5));
//  EXPECT_EQ(720, factorial(6));
//}
