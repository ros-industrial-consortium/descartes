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

#include "descartes_core/trajectory_pt.h"
#include "descartes_core/cart_trajectory_pt.h"
#include "descartes_core/joint_trajectory_pt.h"
#include "ros/console.h"
#include <gtest/gtest.h>

using namespace descartes_core;

// Factory methods for trajectory point construction
template <class T>
TrajectoryPt* CreateTrajectoryPt();

template <>
TrajectoryPt* CreateTrajectoryPt<CartTrajectoryPt>()
{
  return new CartTrajectoryPt();
}

template <>
TrajectoryPt* CreateTrajectoryPt<JointTrajectoryPt>()
{
  return new JointTrajectoryPt();
}

template <class T>
class TrajectoryPtTest : public testing::Test {
 protected:

  TrajectoryPtTest() : lhs_(CreateTrajectoryPt<T>()), rhs_(CreateTrajectoryPt<T>())
  {
  }

  virtual ~TrajectoryPtTest() { delete lhs_; }

  TrajectoryPt* lhs_;
  TrajectoryPt* rhs_;
};

using testing::Types;

// Add types of trajectory points here:
typedef Types<CartTrajectoryPt, JointTrajectoryPt> Implementations;

TYPED_TEST_CASE(TrajectoryPtTest, Implementations);

TYPED_TEST(TrajectoryPtTest, idConstruction) {

  EXPECT_FALSE(this->lhs_->getID().is_nil());
  EXPECT_FALSE(this->rhs_->getID().is_nil());

  //Points (and specifically IDs) should be unique
  EXPECT_NE(this->lhs_, this->rhs_);
  EXPECT_NE(this->lhs_->getID(), this->rhs_->getID());
}

TYPED_TEST(TrajectoryPtTest, idCopy) {
  this->lhs_ = this->rhs_;
  EXPECT_FALSE(this->lhs_->getID().is_nil());
  EXPECT_FALSE(this->rhs_->getID().is_nil());

  //Copies of points (and specifically IDs) should be unique
  EXPECT_EQ(this->lhs_, this->rhs_);
  EXPECT_EQ(this->lhs_->getID(), this->rhs_->getID());
}

