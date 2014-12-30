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
#include <tuple>

namespace descartes_core
{

class SparsePlanner : public descartes_core::PlanningGraph
{

public:
  typedef std::vector<std::tuple<int,TrajectoryPtPtr,JointTrajectoryPt> > SolutionArray;
public:
  SparsePlanner(RobotModelConstPtr &model,double sampling = 0.1f);
  virtual ~SparsePlanner();

  void setSampling(double sampling);
  bool setTrajectoryPoints(const std::vector<TrajectoryPtPtr>& traj);
  bool addTrajectoryPointAfter(TrajectoryPt::ID ref_id,TrajectoryPtPtr cp);
  bool addTrajectoryPointBefore(TrajectoryPt::ID ref_id,TrajectoryPtPtr cp);
  bool modifyTrajectoryPoint(TrajectoryPt::ID ref_id,TrajectoryPtPtr cp);
  const std::map<TrajectoryPt::ID,JointTrajectoryPt>& getSolution();
  bool getSolutionJointPoint(const CartTrajectoryPt::ID& cart_id,JointTrajectoryPt& j);

protected:

  bool plan();
  bool interpolateJointPose(const std::vector<double>& start,const std::vector<double>& end,
                   double t,std::vector<double>& interp);
  int interpolateSparseTrajectory(const SolutionArray& sparse_solution,int &sparse_index, int &point_pos);
  void sampleTrajectory(double sampling,const std::vector<TrajectoryPtPtr>& sparse_trajectory_points,
                        SolutionArray& solution_array);

  int getPointIndex(const TrajectoryPt::ID& ref_id);
  int getSparsePointIndex(const TrajectoryPt::ID& ref_id);

  bool getOrderedSparseTrajectory(std::vector<TrajectoryPtPtr>& sparse_array);

protected:

  enum class InterpolationResult: int
  {
    ERROR = -1,
    REPLAN,
    SUCCESS
  };

  double sampling_;
  std::vector<TrajectoryPtPtr> cart_points_;
  SolutionArray sparse_solution_array_;
  std::map<TrajectoryPt::ID,JointTrajectoryPt> joint_points_map_;

};

} /* namespace descartes_core */
#endif /* SPARSE_PLANNER_H_ */
