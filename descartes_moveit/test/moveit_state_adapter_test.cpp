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

#include "descartes_moveit/moveit_state_adapter.h"
#include <descartes_trajectory_test/robot_model_test.hpp>
#include "moveit/robot_model_loader/robot_model_loader.h"
#include <gtest/gtest.h>

using namespace descartes_moveit;
using namespace descartes_trajectory;
using namespace descartes_core;

using testing::Types;

namespace descartes_trajectory_test
{
// This variable must be global in order for the test to pass.
// Destructing the robot model results in a boost mutex exception:
// ---
// /usr/include/boost/thread/pthread/pthread_mutex_scoped_lock.hpp:26:
// boost::pthread::pthread_mutex_scoped_lock::pthread_mutex_scoped_lock(pthread_mutex_t*):
// Assertion `!pthread_mutex_lock(m)' failed.
// ---

robot_model_loader::RobotModelLoaderPtr robot_;

template <>
RobotModelPtr CreateRobotModel<descartes_moveit::MoveitStateAdapter>()
{
  robot_model::RobotModelPtr moveit_model_;
  robot_state::RobotStatePtr state_;
  descartes_core::RobotModelPtr descartes_model_;

  ROS_INFO_STREAM("Loading robot model from parameter");
  robot_ = robot_model_loader::RobotModelLoaderPtr(
        new robot_model_loader::RobotModelLoader("robot_description"));
  EXPECT_TRUE(robot_);
  ROS_INFO_STREAM("Robot model loaded");
  moveit_model_ = robot_->getModel();
  state_ = robot_state::RobotStatePtr(new robot_state::RobotState(moveit_model_));
  ROS_INFO_STREAM("Construction descartes robot model");
  descartes_model_ = descartes_core::RobotModelPtr(
        new descartes_moveit::MoveitStateAdapter(*state_, "manipulator", "tool0", "base_link"));
  ROS_INFO_STREAM("Descartes robot model constructed");
  return descartes_model_;
}

template<class T>
class MoveitRobotModelTest : public descartes_trajectory_test::RobotModelTest<T>{};

INSTANTIATE_TYPED_TEST_CASE_P(MoveitRobotModelTest, RobotModelTest, MoveitStateAdapter);

} //descartes_moveit_test
