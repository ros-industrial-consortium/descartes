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
/*
 * sparse_planner.h
 *
 *  Created on: Dec 17, 2014
 *      Author: ros developer 
 */

#ifndef SPARSE_PLANNER_H_
#define SPARSE_PLANNER_H_

#include <descartes_core/planning_graph.h>

namespace descartes_core
{

class SparsePlanner : public descartes_core::PlanningGraph
{
public:
  SparsePlanner(RobotModelConstPtr &model);
  virtual ~SparsePlanner();

  bool setTrajectory(const std::vector<TrajectoryPt>& traj, double target_sampling = 0.1f);

public:

  double sampling_;
  std::vector<TrajectoryPt> cart_points_;
  std::map<TrajectoryPt::ID,JointTrajectoryPt> joint_points_map_;

};

} /* namespace descartes_core */
#endif /* SPARSE_PLANNER_H_ */
