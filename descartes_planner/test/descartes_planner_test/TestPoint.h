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

#ifndef TestPoint_H
#define TestPoint_H

#include "descartes_core/robot_model.h"
#include <descartes_trajectory/cart_trajectory_pt.h>
#include <descartes_core/utils.h>

using namespace descartes_core;

namespace descartes_planner_test
{
	class TestPoint: public descartes_trajectory::CartTrajectoryPt
	{
	public:
	  TestPoint(const std::vector<double>& joints);
	  virtual ~TestPoint(){};
	  virtual bool getClosestJointPose(const std::vector<double> &seed_state,
	                                     const RobotModel &model,
	                                     std::vector<double> &joint_pose) const;
	  virtual void getJointPoses(const RobotModel &model,
	                                       std::vector<std::vector<double> > &joint_poses) const;

	protected:

	  std::vector<double> vals_;
	};
}


#endif // TestPoint_H
