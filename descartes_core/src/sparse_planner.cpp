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
 * sparse_planner.cpp
 *
 *  Created on: Dec 17, 2014
 *      Author: ros developer 
 */

#include <descartes_core/sparse_planner.h>

namespace descartes_core
{


SparsePlanner::SparsePlanner(RobotModelConstPtr &model,double sampling):
    PlanningGraph(model),
    sampling_(sampling)
{

}

SparsePlanner::~SparsePlanner()
{

}

void SparsePlanner::setSampling(double sampling)
{
  sampling_ = sampling;
}

bool SparsePlanner::setTrajectoryPoints(const std::vector<TrajectoryPtPtr>& traj)
{
  cart_points_.assign(traj.begin(),traj.end());
  sampleTrajectory(sampling_,cart_points_,sparse_solution_array_);

  // creating temporary sparse array
  std::vector<TrajectoryPtPtr> sparse_traj;
  sparse_traj.reserve(sparse_solution_array_.size());
  for(auto& t : sparse_solution_array_)
  {
    sparse_traj.push_back(std::get<1>(t));
  }

  if(insertGraph(&sparse_traj))
  {

  }
  else
  {
    return false;
  }

  return true;
}

void SparsePlanner::sampleTrajectory(double sampling,const std::vector<TrajectoryPtPtr>& dense_trajectory_points
                                     ,SolutionArray& sparse_trajectory_points )
{
  int skip = dense_trajectory_points.size()/sampling;
  for(int i = 0; i < dense_trajectory_points.size();i+=skip)
  {
    sparse_trajectory_points.push_back(std::make_tuple(i,dense_trajectory_points[i],JointTrajectoryPt()));
  }
}

bool SparsePlanner::interpolateJointPose(const std::vector<double>& start,const std::vector<double>& end,
    double t,std::vector<double>& interp)
{
  if(start.size() != end.size() && (t > 1 || t < 0))
  {
    return false;
  }

  interp.resize(start.size());
  double val = 0.0f;
  for(int i = 0; i < start.size(); i++)
  {
    val = end[i] - (end[i] - start[i]) * (1 - t);
    interp[i] = val;
  }

  return true;
}

bool SparsePlanner::plan()
{
  typedef std::tuple< TrajectoryPt::ID,TrajectoryPt,JointTrajectoryPt > SolutionTuple ;
  std::list<JointTrajectoryPt> sparse_joint_points;
  double cost;

  // solving coarse trajectory
  if(getShortestPath(cost,sparse_joint_points) &&
      (sparse_joint_points.size() == sparse_solution_array_.size()))
  {
    int i = 0;
    for(auto& jp : sparse_joint_points)
    {
      auto& t = sparse_solution_array_[i];
      std::get<2>(t) = jp;
      i++;
    }
    sparse_joint_points.clear();

    // TODO: call interpolateSparseTrajectory here
  }
  else
  {
    return false;
  }



  return true;
}

int SparsePlanner::interpolateSparseTrajectory(const SolutionArray& sparse_solution,int &sparse_index, int &point_pos)
{
  // populating full path
  std::vector<double> start_jpose, end_jpose, rough_interp, aprox_interp, seed_pose(robot_model_->getDOF(),0);
  for(int k = 1; k < sparse_solution.size(); k++)
  {
    auto start_index = std::get<0>(sparse_solution[k-1]);
    auto end_index = std::get<0>(sparse_solution[k]);
    TrajectoryPtPtr start_tpoint = std::get<1>(sparse_solution[k-1]);
    TrajectoryPtPtr end_tpoint = std::get<1>(sparse_solution[k]);
    const JointTrajectoryPt& start_jpoint = std::get<2>(sparse_solution[k-1]);
    const JointTrajectoryPt& end_jpoint = std::get<2>(sparse_solution[k]);

    start_jpoint.getNominalJointPose(seed_pose,*robot_model_,start_jpose);
    end_jpoint.getNominalJointPose(seed_pose,*robot_model_,end_jpose);

    // adding start joint point to solution
    joint_points_map_.insert(std::make_pair(start_tpoint->getID(),start_jpoint));

    // interpolating
    int step = end_index - start_index;
    for(int j = 1; (j < step) && ( (start_index + j) < cart_points_.size()); j++)
    {
      int pos = start_index+j;
      double t = double(j)/double(step);
      if(!interpolateJointPose(start_jpose,end_jpose,t,rough_interp))
      {
        ROS_ERROR_STREAM("Interpolation for point at position "<<pos<< "failed, aborting");
        return (int)InterpolationResult::ERROR;
      }

      TrajectoryPtPtr cart_point = cart_points_[pos];
      if(cart_point->getClosestJointPose(rough_interp,*robot_model_,aprox_interp))
      {
        joint_points_map_.insert(std::make_pair(cart_point->getID(),JointTrajectoryPt(aprox_interp)));
      }
      else
      {
/*        if(addTrajectory(cart_point,start_tpoint->getID(),end_tpoint->getID()) && getShortestPath(cost,sparse_joint_points))
        {
          auto pos_current = std::find(sparse_traj.begin(), sparse_traj.end(),end_tpoint);
          auto pos_new = sparse_traj.insert(pos_current,cart_point);

          // calculating index in joint list
          auto joint_pos = sparse_joint_points.begin();
          std::advance(joint_pos,std::distance(sparse_traj.begin(),pos_new));
          joint_points_map_.insert(std::make_pair(cart_point->getID(),*joint_pos));

          ROS_INFO_STREAM("Graph solved locally for point at position "<<i + j);
        }*/

          sparse_index = k;
          point_pos = pos;
          return (int)InterpolationResult::REPLAN;
      }
    }

    // adding end joint point to solution
    joint_points_map_.insert(std::make_pair(end_tpoint->getID(),start_jpoint));

  }

  return (int)InterpolationResult::SUCCESS;
}

} /* namespace descartes_core */
