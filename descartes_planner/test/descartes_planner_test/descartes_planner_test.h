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

#include <ros/node_handle.h>
#include <descartes_core/utils.h>
#include "descartes_core/pretty_print.hpp"
#include "descartes_core/robot_model.h"
#include "descartes_core/path_planner_base.h"
#include "descartes_planner/sparse_planner.h"
#include "descartes_planner/dense_planner.h"
#include "descartes_planner/planning_graph.h"
#include "ThreeDOFRobot.h"
#include "TestPoint.h"
#include "ros/console.h"
#include <gtest/gtest.h>
#include <tuple>


namespace descartes_planner_test
{

template <class T>
descartes_core::PathPlannerBase* CreateDescartesPlanner();

template <class T>
class DescartesPlannerTest : public ::testing::Test
{
public:
  DescartesPlannerTest() : planner_(CreateDescartesPlanner<T>())
  {
    ROS_INFO("Instantiating DescartesPlannerTest fixture(base) (parameterized)");
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
    ROS_INFO("Destructing DescartesPlannerTest fixture(base) (parameterized)");
  }

  descartes_core::PathPlannerBase* planner_;
};

using namespace descartes_core;
using namespace descartes_planner;
using namespace descartes_trajectory;
typedef std::vector<descartes_core::TrajectoryPtPtr> Trajectory;
const int NUM_DENSE_POINTS = 100;
const int NUM_SPARSE_POINTS = 1000; 
std::string dense = "dense";
std::string sparse = "sparse";
// Trajectory createTestTrajectory();
// Trajectory TEST_TRAJECTORY_SPARSE = createTestTrajectory("sparse");
// Trajectory TEST_TRAJECTORY_SPARSE = createTestTrajectory("dense");

Trajectory createTestTrajectory(std::string const& planner_type)
{ 
  if(planner_type == "dense")
  { 
    ROS_INFO_STREAM("Creating test trajectory with "<<NUM_DENSE_POINTS<<" points");
  }
  else
  {
    ROS_INFO_STREAM("Creating test trajectory with "<<NUM_SPARSE_POINTS<<" points");
  }

  Trajectory traj;
  std::vector<std::tuple<double, double>>joint_bounds = {std::make_tuple(0,M_PI),
                                                         std::make_tuple(-M_PI_2,M_PI_2),
                                                         std::make_tuple(M_PI/8,M_PI/3)};
  std::vector<double> deltas;
  for(auto& e:joint_bounds)
  {
    if(planner_type == "dense")
    {
      double d = (std::get<1>(e)- std::get<0>(e))/NUM_DENSE_POINTS;
      deltas.push_back(d);
    }

    if(planner_type == "sparse")
    {
      double d = (std::get<1>(e)- std::get<0>(e))/NUM_SPARSE_POINTS;
      deltas.push_back(d);
    }
  }

  // creating trajectory points
  std::vector<double> joint_vals(deltas.size(),0);
  if(planner_type =="dense")
  {  
    traj.reserve(NUM_DENSE_POINTS);
    for(int i = 0 ; i < NUM_DENSE_POINTS; i++)
    {
      for(int j = 0; j < deltas.size(); j++)
      {
        joint_vals[j] = std::get<0>(joint_bounds[j]) + deltas[j]*i;
      }

      TrajectoryPtPtr tp(new TestPoint(joint_vals));
      traj.push_back(tp);
    }
  } 
  if(planner_type =="sparse")
  {  
    traj.reserve(NUM_SPARSE_POINTS);
    for(int i = 0 ; i < NUM_SPARSE_POINTS; i++)
    {
      for(int j = 0; j < deltas.size(); j++)
      {
        joint_vals[j] = std::get<0>(joint_bounds[j]) + deltas[j]*i;
      }

      TrajectoryPtPtr tp(new TestPoint(joint_vals));
      traj.push_back(tp);
    }
  }
  return traj;
}

TYPED_TEST_CASE_P(DescartesPlannerTest);

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
  RobotModelConstPtr robot(new ThreeDOFRobot());
  this->planner_->initialize(robot);
  if(dynamic_cast<SparsePlanner*>(this->planner_))
  { 
    ROS_INFO_STREAM("Testing planPath() with "<<NUM_SPARSE_POINTS<<" points");
    EXPECT_TRUE(this->planner_->planPath(createTestTrajectory(sparse)));
  }  
  if(dynamic_cast<DensePlanner*>(this->planner_))
  { 
    ROS_INFO_STREAM("Testing planPath() with "<<NUM_DENSE_POINTS<<" points");
    EXPECT_TRUE(this->planner_->planPath(createTestTrajectory(dense)));
  }
}

TYPED_TEST_P(DescartesPlannerTest, getPath)
{
  std::vector<descartes_core::TrajectoryPtPtr> path;
  RobotModelConstPtr robot(new ThreeDOFRobot());
  this->planner_->initialize(robot);
  if(dynamic_cast<SparsePlanner*>(this->planner_))
  { 
    EXPECT_TRUE(this->planner_->planPath(createTestTrajectory(sparse)));
    EXPECT_TRUE(this->planner_->getPath(path));
    EXPECT_TRUE(path.size() == NUM_SPARSE_POINTS); 
  }  
  if(dynamic_cast<DensePlanner*>(this->planner_))
  { 
    EXPECT_TRUE(this->planner_->planPath(createTestTrajectory(dense)));
    EXPECT_TRUE(this->planner_->getPath(path));
    EXPECT_TRUE(path.size() == NUM_DENSE_POINTS);
  }
}

REGISTER_TYPED_TEST_CASE_P(DescartesPlannerTest, initialize, configure, planPath, getPath);

}

#endif // DESCARTES_PLANNER_H