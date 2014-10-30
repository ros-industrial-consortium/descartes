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
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "ros/console.h"
#include <gtest/gtest.h>


using namespace descartes_moveit;

class RobotModelTest : public ::testing::Test {
 protected:
  virtual void SetUp() {

    ROS_INFO_STREAM("Loading robot model from parameter");
    robot_ = robot_model_loader::RobotModelLoaderPtr(
          new robot_model_loader::RobotModelLoader("robot_description"));
    moveit::core::RobotState state(robot_->getModel());
    model_ = descartes_core::RobotModelPtr(new descartes_moveit::MoveitStateAdapter(state, "manipulator",
                                                                                    "tool0", "base_link"));

  }

  // virtual void TearDown() {}

  robot_model_loader::RobotModelLoaderPtr robot_;
  descartes_core::RobotModelPtr model_;

};


TEST_F(RobotModelTest, construction) {
  EXPECT_TRUE(true);
  EXPECT_TRUE(true);
}
