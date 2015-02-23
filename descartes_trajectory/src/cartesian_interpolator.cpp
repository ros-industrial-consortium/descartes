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

#include <console_bridge/console.h>
#include <descartes_trajectory/cartesian_interpolator.h>
#include <tf_conversions/tf_eigen.h>
#include <boost/uuid/uuid_io.hpp>
#include <math.h>


#define NOT_IMPLEMENTED_ERR(ret) logError("%s not implemented", __PRETTY_FUNCTION__); return ret;
const static unsigned int MIN_NUM_STEPS = 1;

using namespace descartes_core;

namespace descartes_trajectory
{
LinearSegment::LinearSegment(const tf::Transform& start_pose,
                             const tf::Transform& end_pose,
                             double total_time,
                             double interpolation_step):
                             start_pose_(start_pose),
                             end_pose_(end_pose),
                             total_time_(total_time),
                             interpolation_step_(interpolation_step)
{

}

LinearSegment::~LinearSegment()
{

}

bool LinearSegment::interpolate(std::vector<descartes_core::TrajectoryPtPtr>& segment_points)
{

  tf::Vector3 vel = (end_pose_.getOrigin() - start_pose_.getOrigin())/total_time_;
  double time_step = interpolation_step_/total_time_;
  std::size_t num_steps = std::ceil(total_time_/interpolation_step_)-1;

  if(num_steps <= MIN_NUM_STEPS)
  {
    ROS_ERROR_STREAM("Interpolation steps value of "<<num_steps<<" is invalid");
    return false;
  }
  segment_points.clear();

  segment_points.reserve(num_steps);
  tf::Transform p;
  Eigen::Affine3d eigen_pose;
  tf::Quaternion q;
  for(std::size_t i = 1; i < num_steps; i++)
  {
    p.setOrigin(start_pose_.getOrigin() +  vel*i*interpolation_step_);
    start_pose_.getRotation().slerp(q,i*time_step);
    tf::poseTFToEigen(p,eigen_pose);
    segment_points.push_back(TrajectoryPtPtr(new CartTrajectoryPt(eigen_pose)));
  }

  ROS_DEBUG_STREAM("Segment contains "<<segment_points.size()<<" interpolated points for "<<num_steps<<" time steps");

  return true;
}

CartesianInterpolator::CartesianInterpolator()
{

}

CartesianInterpolator::~CartesianInterpolator()
{

}

bool CartesianInterpolator::computeBlendSegmentTimes(const std::vector<descartes_trajectory::CartTrajectoryPt>& coarse_traj,
                                                     std::vector<double>& bt)
{
  NOT_IMPLEMENTED_ERR(false);
}

bool CartesianInterpolator::computeFullSegmentTimes(const std::vector<descartes_trajectory::CartTrajectoryPt>& coarse_traj,
                                                    std::vector<double>& ft)
{
  ft.resize(coarse_traj.size() - 1);
  Eigen::Affine3d p_start, p_end;
  std::vector<double> seed(robot_model_->getDOF(),0.0f);
  for(unsigned int i = 1; i < coarse_traj.size();i++)
  {
    const CartTrajectoryPt& c1 = coarse_traj[i-1];
    const CartTrajectoryPt& c2 = coarse_traj[i];
    if(c1.getNominalCartPose(seed,*robot_model_,p_start) &&
        c2.getNominalCartPose(seed,*robot_model_,p_end))
    {

      Eigen::Vector3d p1 = p_start.translation();
      Eigen::Vector3d p2 = p_end.translation();
      double norm = (p2-p1).norm();

      ROS_DEBUG_STREAM("Distance for segment "<<i-1<<": "<<(p2-p1).norm());
      double dt = norm/tool_speed_;
      ft[i-1] = dt;
    }
    else
    {
      ROS_ERROR_STREAM("Failed to get nominal cartesian poses while calculating time for segment "
          <<c1.getID()<<" and "<<c2.getID());
      return false;
    }
  }

  return true;
}

bool CartesianInterpolator::initialize(descartes_core::RobotModelConstPtr robot_model,
                                       double tool_speed,double interpolation_interval, double zone_radius)
{
  robot_model_ = robot_model;
  tool_speed_ = tool_speed;
  interpolation_interval_= interpolation_interval;
  zone_radius_ = zone_radius;

  return true;
}

bool CartesianInterpolator::interpolate(const std::vector<descartes_trajectory::CartTrajectoryPt>& coarse_traj,
                 std::vector<descartes_core::TrajectoryPtPtr>& interpolated_traj)
{

  std::vector<double> segment_times;
  if(!computeFullSegmentTimes(coarse_traj,segment_times))
  {
    ROS_ERROR_STREAM("Failed to compute trajectory segment time values");
    return false;
  }

  // estimating number of total points in interpolated trajectory
  std::size_t total_points = 0;
  for(std::size_t i = 0; i < segment_times.size();i++)
  {
    total_points += std::ceil(segment_times[i]/interpolation_interval_) + 1;
  }
  interpolated_traj.clear();
  interpolated_traj.reserve(total_points);

  // interpolating segments

   tf::Transform p_start, p_end;
  Eigen::Affine3d eigen_start, eigen_end;
  std::vector<double> seed(robot_model_->getDOF(),0.0f);
  std::vector<TrajectoryPtPtr> segment_points;
  for(std::size_t i = 0; i < segment_times.size();i++)
  {
    const CartTrajectoryPt& ps = coarse_traj[i];
    const CartTrajectoryPt& pe = coarse_traj[i+1];
    ps.getNominalCartPose(seed,*robot_model_,eigen_start);
    pe.getNominalCartPose(seed,*robot_model_,eigen_end);

    tf::poseEigenToTF(eigen_start,p_start);
    tf::poseEigenToTF(eigen_end,p_end);
    LinearSegment lsegment = LinearSegment(p_start,p_end,segment_times[i],interpolation_interval_);
    if(lsegment.interpolate(segment_points))
    {
      interpolated_traj.push_back(TrajectoryPtPtr(new CartTrajectoryPt(ps)));
      interpolated_traj.insert(interpolated_traj.end(), segment_points.begin(),segment_points.end());

      if(i == segment_times.size() - 1)
      {
        interpolated_traj.push_back(TrajectoryPtPtr(new CartTrajectoryPt(pe)));
      }

      segment_points.clear();
    }
    else
    {
      ROS_ERROR_STREAM("Linear interpolation for segment "<<i<<" failed");
      return false;
    }
  }

  return true;
}

} /* namespace descartes_trajectory */
