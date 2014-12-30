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
#include <boost/uuid/uuid_io.hpp>
#include <algorithm>

namespace descartes_core
{

const int MAX_REPLANNING_ATTEMPTS = 100;

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
  ROS_INFO_STREAM("Sampled trajectory contains "<<sparse_solution_array_.size()<<" points from "<<cart_points_.size()<<
                  " points in the dense trajectory");

  // creating temporary sparse array
  std::vector<TrajectoryPtPtr> sparse_traj;
  sparse_traj.reserve(sparse_solution_array_.size());
  for(auto& t : sparse_solution_array_)
  {
    sparse_traj.push_back(std::get<1>(t));
  }

  if(insertGraph(&sparse_traj) && plan())
  {
    int planned_count = sparse_solution_array_.size();
    int interp_count = cart_points_.size()  - sparse_solution_array_.size();
    ROS_INFO("Sparse plan succeeded with %i planned point and %i interpolated points",planned_count,interp_count);
  }
  else
  {
    return false;
  }

  return true;
}

bool SparsePlanner::addTrajectoryPointAfter(TrajectoryPt::ID ref_id,TrajectoryPtPtr cp)
{
  // finding position
  auto predicate = [ref_id](TrajectoryPtPtr cp)
    {
      return ref_id== cp->getID();
    };
  auto pos = std::find_if(cart_points_.begin(),cart_points_.end(),predicate);
  if(pos == cart_points_.end())
  {
    ROS_ERROR_STREAM("Point with ID "<<ref_id<<" was not found, aborting");
    return false;
  }

  int index = std::distance(cart_points_.begin(),pos);
  TrajectoryPt::ID prev_id, next_id;

  if(index == cart_points_.size() - 1) // last element
  {
    prev_id = cart_points_.back()->getID();
    next_id = boost::uuids::nil_uuid();

    // inserting new point into sparse trajectory array
    sparse_solution_array_.push_back(std::make_tuple(index,cp,JointTrajectoryPt()));
  }
  else
  {
    prev_id = cart_points_[index]->getID();
    next_id = cart_points_[index+1]->getID();

    // inserting new point into sparse trajectory array
    auto sparse_pos = std::find_if(sparse_solution_array_.begin(),sparse_solution_array_.end(),
                                   [index](std::tuple<int,TrajectoryPtPtr,JointTrajectoryPt>& t)
                                         {
                                           return index < std::get<0>(t);
                                         });
    if(sparse_pos != sparse_solution_array_.end())
    {
      sparse_solution_array_.insert(sparse_pos,std::make_tuple(index,cp,JointTrajectoryPt()));
    }
    else
    {
      ROS_ERROR_STREAM("Point with ID "<<ref_id<<" could not be added to sparse trajectory array, aborting");
      return false;
    }
  }


  // replanning
  if(addTrajectory(cp,prev_id,next_id) && plan())
  {
    int planned_count = sparse_solution_array_.size();
    int interp_count = cart_points_.size()  - sparse_solution_array_.size();
    ROS_INFO("Sparse plan succeeded with %i planned point and %i interpolated points",planned_count,interp_count);
  }
  else
  {
    return false;
  }

  return true;
}

bool SparsePlanner::addTrajectoryPointBefore(TrajectoryPt::ID ref_id,TrajectoryPtPtr cp)
{
  // finding position
  auto predicate = [ref_id](TrajectoryPtPtr cp)
    {
      return ref_id== cp->getID();
    };
  auto pos = std::find_if(cart_points_.begin(),cart_points_.end(),predicate);
  if(pos == cart_points_.end())
  {
    ROS_ERROR_STREAM("Reference ID "<<ref_id<<" was not found, aborting");
    return false;
  }

  int index = std::distance(cart_points_.begin(),pos);
  TrajectoryPt::ID prev_id, next_id;

  if(index == 0) // first element
  {
    prev_id = boost::uuids::nil_uuid();
    next_id = cart_points_.front()->getID();

    // inserting new point into sparse trajectory array
    sparse_solution_array_.insert(sparse_solution_array_.begin(),std::make_tuple(index,cp,JointTrajectoryPt()));
  }
  else
  {
    prev_id = cart_points_[index-1]->getID();
    next_id = cart_points_[index]->getID();

    // inserting new point into sparse trajectory array
    auto sparse_pos = std::find_if(sparse_solution_array_.begin(),sparse_solution_array_.end(),
                                   [index](std::tuple<int,TrajectoryPtPtr,JointTrajectoryPt>& t)
                                         {
                                           return index < std::get<0>(t);
                                         });
    if(sparse_pos != sparse_solution_array_.end())
    {
      sparse_solution_array_.insert(sparse_pos,std::make_tuple(index,cp,JointTrajectoryPt()));
    }
    else
    {
      ROS_ERROR_STREAM("Point with ID "<<ref_id<<" could not be added to sparse trajectory array, aborting");
      return false;
    }
  }

  if(addTrajectory(cp,prev_id,next_id) && plan())
  {
    int planned_count = sparse_solution_array_.size();
    int interp_count = cart_points_.size()  - sparse_solution_array_.size();
    ROS_INFO("Sparse plan succeeded with %i planned point and %i interpolated points",planned_count,interp_count);
  }
  else
  {
    return false;
  }

  return true;
}

int SparsePlanner::getPointIndex(const TrajectoryPt::ID& ref_id)
{
  int index = -1;
  auto predicate = [ref_id](TrajectoryPtPtr cp)
    {
      return ref_id== cp->getID();
    };

  auto pos = std::find_if(cart_points_.begin(),cart_points_.end(),predicate);
  if(pos == cart_points_.end())
  {
    index = std::distance(cart_points_.begin(),pos);
  }

  return index;
}

int SparsePlanner::getSparsePointIndex(const TrajectoryPt::ID& ref_id)
{
  int index = -1;
  auto predicate = [ref_id](std::tuple<int,TrajectoryPtPtr,JointTrajectoryPt>& t)
    {
      return ref_id == std::get<1>(t)->getID();
    };

  auto pos = std::find_if(sparse_solution_array_.begin(),sparse_solution_array_.end(),predicate);
  if(pos != sparse_solution_array_.end())
  {
    index = std::distance(sparse_solution_array_.begin(),pos);
  }

  return index;
}

bool SparsePlanner::getOrderedSparseTrajectory(std::vector<TrajectoryPtPtr>& sparse_array)
{
  const CartesianMap& cart_map = getCartesianMap();
  TrajectoryPt::ID first_id = boost::uuids::nil_uuid();
  auto predicate = [&first_id](const std::pair<TrajectoryPt::ID,CartesianPointInformation>& p)
    {
      const auto& info = p.second;
      if(info.links_.id_previous == boost::uuids::nil_uuid())
      {
        first_id = p.first;
        return true;
      }
      else
      {
        return false;
      }
    };



  if(cart_map.empty()
      || (std::find_if(cart_map.begin(),cart_map.end(),predicate) == cart_map.end())
      || first_id == boost::uuids::nil_uuid())
  {
    return false;
  }

  // copying point pointers in order
  sparse_array.resize(cart_map.size());
  TrajectoryPt::ID current_id = first_id;
  for(int i = 0; i < sparse_array.size(); i++)
  {
    if(cart_map.count(current_id) == 0)
    {
      return false;
    }

    const CartesianPointInformation& info  = cart_map.at(current_id);
    sparse_array[i] = info.source_trajectory_;
    current_id = info.links_.id_next;
  }

  return true;
}

bool SparsePlanner::modifyTrajectoryPoint(TrajectoryPt::ID ref_id,TrajectoryPtPtr cp)
{
  int index = getPointIndex(ref_id);
  if(index < 0)
  {
    return false;
  }
  // copying cartesian point
  cp->setID(cart_points_[index]->getID());
  cart_points_[index] = cp;

  // copying point in sparse trajectory array if its there
  int sparse_index = getSparsePointIndex(ref_id);
  if(sparse_index < 0)
  {
    return false;
  }

  sparse_solution_array_[sparse_index] = std::make_tuple(index,cp,JointTrajectoryPt());

  return true;
}

bool SparsePlanner::getSolutionJointPoint(const CartTrajectoryPt::ID& cart_id, JointTrajectoryPt& j)
{
  if(joint_points_map_.count(cart_id) > 0)
  {
    j = joint_points_map_[cart_id];
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
  bool replan = true;
  bool succeeded = false;
  int replanning_attempts = 0;
  while(replan && (replanning_attempts++ < MAX_REPLANNING_ATTEMPTS)
      && getShortestPath(cost,sparse_joint_points) &&
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
    int sparse_index, point_pos;
    int result = interpolateSparseTrajectory(sparse_solution_array_,sparse_index,point_pos);
    TrajectoryPt::ID prev_id, next_id;
    TrajectoryPtPtr cart_point;
    auto sparse_iter = sparse_solution_array_.begin();
    switch(result)
    {
      case int(InterpolationResult::REPLAN):
          replan = true;
          cart_point = cart_points_[point_pos];
          prev_id = cart_points_[point_pos - 1]->getID();
          next_id = cart_points_[point_pos + 1]->getID();
          std::advance(sparse_iter,sparse_index);
          sparse_solution_array_.insert(sparse_iter,std::make_tuple(point_pos,cart_point,JointTrajectoryPt()));
          if(addTrajectory(cart_point,prev_id,next_id))
          {
            ROS_INFO_STREAM("Added new point to sparse trajectory from dense trajectory at position "<<
                            point_pos<<", re-planning entire trajectory");
          }
          else
          {
            ROS_INFO_STREAM("Adding point "<<point_pos <<"to sparse trajectory failed, aborting");
            replan = false;
            succeeded = false;
          }

          break;
      case int(InterpolationResult::SUCCESS):
          replan = false;
          succeeded = true;
          break;
      case int(InterpolationResult::ERROR):
          replan = false;
          succeeded = false;
          break;
    }

  }

  return true;
}

int SparsePlanner::interpolateSparseTrajectory(const SolutionArray& sparse_solution,int &sparse_index, int &point_pos)
{
  // populating full path
  joint_points_map_.clear();
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
