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
 * trajectory_segment.h
 *
 *  Created on: Mar 26, 2015
 *      Author: Jorge Nicho
 */

#include <descartes_trajectory/trajectory_segment.h>
#include <tf_conversions/tf_eigen.h>
#include <boost/uuid/uuid_io.hpp>
#include <math.h>

const unsigned int MIN_NUM_STEPS = 1;
const double ZERO = 0.0f;

using namespace descartes_core;

namespace descartes_trajectory
{

LinearSegment::LinearSegment(const Eigen::Affine3d& start_pose,
                             const Eigen::Affine3d& end_pose,
                             double duration,
                             double time_step,
                             const std::pair<double,double>& interval):
                             start_pose_(start_pose),
                             end_pose_(end_pose),
                             duration_(duration),
                             time_step_(time_step),
                             interval_(interval)
{

}

LinearSegment::~LinearSegment()
{

}

bool LinearSegment::interpolate(std::vector<descartes_core::TrajectoryPtPtr>& segment_points)
{

  if( duration_<= ZERO )
  {
    ROS_ERROR_STREAM("The values for 'duration'  must be > 0, a negative number was passed");
    return false;
  }

  if(time_step_ <= ZERO )
  {
    ROS_ERROR_STREAM("The values for 'time_step' must be > 0, a negative number was passed");
    return false;
  }

  if(duration_ <= time_step_)
  {
    ROS_ERROR("The segment duration = %f can not be less than the interpolation time step = %f",duration_,time_step_);
    return false;
  }

  if(interval_.first < 0)
  {
    ROS_ERROR_STREAM("Interval must start at a value >= 0 , an incorrect value of "<<interval_.first<<" was passed");
    return false;
  }

  if((interval_.second - interval_.first) < time_step_)
  {
    ROS_ERROR_STREAM("The difference between the start and end of the interval is smaller than the time step, ("<<
                     interval_.second<<" - "<<interval_.first<<") < "<<time_step_);
    return false;
  }


  // conversions to tf
  tf::Transform end_pose_tf, start_pose_tf;
  tf::poseEigenToTF(start_pose_,start_pose_tf);
  tf::poseEigenToTF(end_pose_,end_pose_tf);

  tf::Vector3 vel = (end_pose_tf.getOrigin() - start_pose_tf.getOrigin())/duration_;
  std::size_t num_steps = std::floor(duration_/time_step_ + 1)-1;
  ROS_DEBUG_STREAM("LINEAR SEGMENT DURATION: "<<duration_ <<", TIME_STEP: "<<time_step_ <<", NUM_STEPS: "<<num_steps<<
                   " INTERVAL: ["<<interval_.first<<", "<<interval_.second<<"]");

  if(num_steps <= MIN_NUM_STEPS)
  {
    ROS_ERROR_STREAM("Interpolation steps value of "<<num_steps<<" is less that the minimum allowed "<<MIN_NUM_STEPS);
    return false;
  }

  segment_points.clear();
  segment_points.reserve(num_steps+1);

  tf::Transform p;
  Eigen::Affine3d eigen_pose;
  double dt = 0;
  for(std::size_t i = 0; i <= num_steps; i++)
  {

    dt = i*time_step_;

    if(dt < interval_.first)
    {
      if((interval_.first -     dt) < time_step_)
      {
        dt = interval_.first;
      }
      else
      {
        continue;
      }
    }

    if(dt > interval_.second)
    {
      if( (dt - interval_.second) < time_step_)
      {
        dt = interval_.second;
      }
      else
      {
        break;
      }
    }

    p.setOrigin(start_pose_tf.getOrigin() +  vel*dt);
    p.setRotation(start_pose_tf.getRotation().slerp(end_pose_tf.getRotation(),dt/duration_));
    tf::poseTFToEigen(p,eigen_pose);
    segment_points.push_back(TrajectoryPtPtr(new CartTrajectoryPt(eigen_pose)));
  }

  ROS_DEBUG_STREAM("Segment contains "<<segment_points.size()<<" interpolated points for "<<num_steps<<" time steps");

  return true;
}

BlendSegment::BlendSegment(const Eigen::Affine3d& start_pose,
                           const Eigen::Affine3d& end_pose,
               const Eigen::Vector3d& start_vel,
               const Eigen::Vector3d& end_vel,
               double time_step):
                   start_pose_(start_pose),
                   end_pose_(end_pose),
                   start_vel_(start_vel),
                   end_vel_(end_vel),
                   time_step_(time_step)
{
  Eigen::Vector3d start_pos = start_pose_.translation();
  Eigen::Vector3d end_pos = end_pose_.translation();

  // determining segment duration
  duration_ = 2*((end_pos - start_pos).norm())/((start_vel + end_vel).norm());

  // determining linear acceleration vector
  accel_ = (end_vel - start_vel)/duration_;
}

BlendSegment::~BlendSegment()
{

}

bool BlendSegment::interpolate(std::vector<descartes_core::TrajectoryPtPtr>& segment_points)
{
  if( duration_<= ZERO )
  {
    ROS_ERROR_STREAM("The blend segment duration must be >= 0, it was computed to 2*(Pf - P0)/(Vf + V0): "<<duration_);
    return false;
  }

  if(time_step_ <= ZERO )
  {
    ROS_ERROR_STREAM("The values for 'time_step' must be >= 0, a negative number was passed");
    return false;
  }

  if(duration_ <= time_step_)
  {
    ROS_ERROR("The blend segment duration = %f can not be less than the interpolation time step = %f",duration_,time_step_);
    return false;
  }

  // determining number of points
  unsigned int num_steps = std::floor(duration_/time_step_ + 1 ) -1;
  segment_points.clear();
  segment_points.reserve(num_steps+1);
  ROS_DEBUG_STREAM("BLEND SEGMENT DURATION: "<<duration_<<", TIME_STEP: "<<time_step_ <<", NUM_STEPS: "<<num_steps
                   <<", ACCEL: ["<<accel_(0)<<", "<<accel_(1)<<", "<<accel_(2)<<"]");

  tf::Transform p;
  Eigen::Affine3d eigen_pose;
  tf::Transform end_pose_tf, start_pose_tf;
  tf::poseEigenToTF(start_pose_,start_pose_tf);
  tf::poseEigenToTF(end_pose_,end_pose_tf);
  Eigen::Vector3d vel;
  Eigen::Vector3d pos, pos0;
  tf::vectorTFToEigen(start_pose_tf.getOrigin(),pos0);
  double t;

  for(unsigned int i = 0 ; i <= num_steps; i++)
  {
    t = i * time_step_;
    t = (t > duration_ ) ? duration_ : t;


    vel = start_vel_ + accel_ * t;
    pos = pos0 + start_vel_*t + 0.5f*accel_*std::pow(t,2);

    p.setOrigin(tf::Vector3(pos(0),pos(1),pos(2)));
    p.setRotation(start_pose_tf.getRotation().slerp(end_pose_tf.getRotation(),t/duration_));

    tf::poseTFToEigen(p,eigen_pose);
    segment_points.push_back(TrajectoryPtPtr(new CartTrajectoryPt(eigen_pose)));
  }

  return true;
}

}
