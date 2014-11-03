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
#include "console_bridge/console.h"
#include "ros/console.h"
#include <gtest/gtest.h>
#include <iostream>


using namespace descartes_moveit;

class RobotModelTest : public ::testing::Test {
 protected:
  virtual void SetUp() {

    ROS_INFO_STREAM("Loading robot model from parameter");
    robot_ = robot_model_loader::RobotModelLoaderPtr(
          new robot_model_loader::RobotModelLoader("robot_description"));
    ASSERT_TRUE(robot_);
    ROS_INFO_STREAM("Robot model loaded");
    moveit_model_ = robot_->getModel();
    state_ = robot_state::RobotStatePtr(new robot_state::RobotState(moveit_model_));
    ROS_INFO_STREAM("Construction descartes robot model");
    descartes_model_ = descartes_core::RobotModelPtr(new descartes_moveit::MoveitStateAdapter(*state_, "manipulator",
                                                                                    "tool0", "base_link"));
    ROS_INFO_STREAM("Descartes robot model constructed");
  }

  // virtual void TearDown() {}

  robot_model_loader::RobotModelLoaderPtr robot_;
  robot_model::RobotModelPtr moveit_model_;
  robot_state::RobotStatePtr state_;
  descartes_core::RobotModelPtr descartes_model_;

};

bool equal(std::vector<double> lhs, std::vector<double> rhs, double tol)
{
  bool rtn = false;
  if( lhs.size() == rhs.size() )
  {
    rtn = true;
    for(size_t ii = 0; ii < lhs.size(); ++ii)
    {
      if(std::fabs(lhs[ii]-rhs[ii]) > tol)
      {
        ROS_INFO_STREAM("Vectors not equal, index: " << ii << ", lhs: "
                        << lhs[ii] << ", rhs: " << rhs[ii]);
        rtn = false;
        break;
      }
    }

  }
  else
  {
    ROS_INFO_STREAM("Vectors not equal, different sizes: lhs: " << lhs.size()
                    << ",rhs: " << rhs.size());
    rtn = false;
  }
  return rtn;
}

const double TF_EQ_TOL = 0.001;
const double JOINT_EQ_TOL = 0.001;

TEST_F(RobotModelTest, construction) {
  ROS_INFO_STREAM("Robot model test construction");
}


TEST_F(RobotModelTest, getIK) {
  ROS_INFO_STREAM("Testing getIK");
  std::vector<double> fk_joint(6, 0.5);
  std::vector<double> ik_joint;
  Eigen::Affine3d ik_pose, fk_pose;
  EXPECT_TRUE(this->descartes_model_->getFK(fk_joint, ik_pose));
  EXPECT_TRUE(this->descartes_model_->getIK(ik_pose, fk_joint, ik_joint));
  //This doesn't always work, but it should.  For some reason the getIK method
  //does not return the "nearest" solution like it should when joint position
  //of all zeros is used.
  EXPECT_TRUE(equal(fk_joint, ik_joint, JOINT_EQ_TOL));
  EXPECT_TRUE(this->descartes_model_->getFK(ik_joint, fk_pose));
  EXPECT_TRUE(ik_pose.matrix().isApprox(fk_pose.matrix(), TF_EQ_TOL));
}

TEST_F(RobotModelTest, getAllIK) {
  ROS_INFO_STREAM("Testing getAllIK");
  ROS_DEBUG_STREAM("Debug message");
  std::vector<double> fk_joint(6, 0.5);
  std::vector<std::vector<double> > joint_poses;
  Eigen::Affine3d ik_pose, fk_pose;

  EXPECT_TRUE(this->descartes_model_->getFK(fk_joint, ik_pose));
  EXPECT_TRUE(this->descartes_model_->getAllIK(ik_pose, joint_poses));
  ROS_INFO_STREAM("Get all IK returned " << joint_poses.size() << " solutions");
  std::vector<std::vector<double> >::iterator it;
  for (it = joint_poses.begin(); it != joint_poses.end(); ++it)
  {
    EXPECT_TRUE(this->descartes_model_->getFK(*it, fk_pose));
    EXPECT_TRUE(ik_pose.matrix().isApprox(fk_pose.matrix(), TF_EQ_TOL));
  }
}
