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
#include "ros/console.h"
#include <gtest/gtest.h>

TYPED_TEST_P(descartes_planner, initialize)
{
  ros::Time::init();
  RobotModelConstPtr robot(new ThreeDOFRobot());
  EXPECT_TRUE(Planner.initialize(robot));
}

TYPED_TEST_P(descartes_planner, configure)
{
  descartes_core::PlannerConfig config;
  Planner.getConfig(config);
  EXPECT_TRUE(Planner.setConfig(config));
}


TYPED_TEST_P(descartes_planner, planPath)
{
  ROS_INFO_STREAM("Testing planPath() with "<<NUM_DENSE_POINTS<<" points");
  EXPECT_TRUE(Planner.planPath(TEST_TRAJECTORY));
}

TYPED_TEST_P(descartes_planner, getPath)
{
  std::vector<descartes_core::TrajectoryPtPtr> path;
  EXPECT_TRUE(Planner.getPath(path));
  EXPECT_TRUE(path.size() == NUM_DENSE_POINTS);
}

REGISTER_TYPED_TEST_CASE_P(descartes_planner, initialize, configure, planPath, getPath);

#endif DESCARTES_PLANNER_H