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

#ifndef DESCARTES_PLANNER_H
#define DESCARTES_PLANNER_H

#include "descartes_core/pretty_print.hpp"
#include "descartes_core/robot_model.h"
#include <descartes_core/path_planner_base.h>
#include "ros/console.h"
#include <gtest/gtest.h>

namespace descartes_planner_test
{

template <class T>
descartes_planner CreateDescartesPlanner();

template <class T>
class DescartesPlannerTest : public ::testing::Test
{
public:
  DescartesPlannerTest() : planner_(CreateDescartesPlanner<T>())
  {
    ROS_INFO("Instantiated DescartesPlannerTest fixture(base) (parameterized)");
  }

  virtual void SetUp()
  {
    ROS_INFO("Setting up DescartesPlannerTest fixture(base) (parameterized)");
    ASSERT_TRUE(static_cast<bool>(this->planner_));
  }

  virtual void TearDown()
  {
    ROS_INFO("Tearing down DescartesPlannerTest fixture(base) (parameterized)");
  }

  virtual ~DescartesPlannerTest()
  {
    ROS_INFO("Desctructing DescartesPlannerTest fixture(base) (parameterized)");
  }

  descartes_core::PathPlannerBase planner_;
};

using namespace descartes_core;

TYPED_TEST_P(DescartesPlannerTest, initialize)
{
  ros::Time::init();
  RobotModelConstPtr robot(new ThreeDOFRobot());
  EXPECT_TRUE(this->planner_->initialize(robot));
}

TYPED_TEST_P(DescartesPlannerTest, configure)
{
  descartes_core::PlannerConfig config;
  this->planner_->getConfig(config);
  EXPECT_TRUE(this->planner_->setConfig(config));
}


TYPED_TEST_P(DescartesPlannerTest, planPath)
{
  ROS_INFO_STREAM("Testing planPath() with "<<NUM_DENSE_POINTS<<" points");
  EXPECT_TRUE(this->planner_->planPath(TEST_TRAJECTORY));
}

TYPED_TEST_P(DescartesPlannerTest, getPath)
{
  std::vector<descartes_core::TrajectoryPtPtr> path;
  EXPECT_TRUE(this->planner_->getPath(path));
  EXPECT_TRUE(path.size() == NUM_DENSE_POINTS);
}

REGISTER_TYPED_TEST_CASE_P(DescartesPlannerTest, initialize, configure, planPath, getPath);

}
#endif // DESCARTES_PLANNER_H