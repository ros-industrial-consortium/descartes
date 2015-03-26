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

#include <descartes_trajectory/cartesian_interpolator.h>
#include <tf_conversions/tf_eigen.h>
#include <boost/uuid/uuid_io.hpp>
#include <math.h>


#define NOT_IMPLEMENTED_ERR(ret) ROS_ERROR("%s not implemented", __PRETTY_FUNCTION__); return ret;
const static unsigned int MIN_NUM_STEPS = 1;
const double ZERO = 0.0f;
const double EPSILON = 1.0e-10;

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

CartesianInterpolator::CartesianInterpolator():
    tool_speed_(0),
    interpolation_interval_(0),
    zone_radius_(0),
    robot_model_()
{

}

CartesianInterpolator::~CartesianInterpolator()
{

}

bool CartesianInterpolator::computeFullSegmentTimes(const std::vector<descartes_core::TrajectoryPtPtr>& coarse_traj,
                                                    std::vector<double>& ft)
{
  ft.resize(coarse_traj.size() - 1);
  Eigen::Affine3d p_start, p_end;
  std::vector<double> seed(robot_model_->getDOF(),0.0f);
  for(unsigned int i = 1; i < coarse_traj.size();i++)
  {
    descartes_core::TrajectoryPtPtr c1 = coarse_traj[i-1];
    descartes_core::TrajectoryPtPtr c2 = coarse_traj[i];
    if(c1->getNominalCartPose(seed,*robot_model_,p_start) &&
        c2->getNominalCartPose(seed,*robot_model_,p_end))
    {

      Eigen::Vector3d p1 = p_start.translation();
      Eigen::Vector3d p2 = p_end.translation();
      double norm = (p2-p1).norm();

      ROS_DEBUG_STREAM("Distance for segment "<<i-1<<": "<<(p2-p1).norm());
      double dt = norm/tool_speed_;
      ft[i-1] = dt;

      ROS_DEBUG_STREAM("INTERVAL["<<i-1<<"] time "<<dt);
    }
    else
    {
      ROS_ERROR_STREAM("Failed to get nominal cartesian poses while calculating time for segment "
          <<c1->getID()<<" and "<<c2->getID());
      return false;
    }
  }

  return true;
}

bool CartesianInterpolator::initialize(descartes_core::RobotModelConstPtr robot_model,
                                       double tool_speed,double interpolation_interval, double zone_radius)
{
  robot_model_ = robot_model;
  tool_speed_ = std::abs(tool_speed);
  interpolation_interval_= std::abs(interpolation_interval);
  zone_radius_ = std::abs(zone_radius);

  return true;
}

bool CartesianInterpolator::interpolate(const std::vector<descartes_core::TrajectoryPtPtr>& coarse_traj,
                 std::vector<descartes_core::TrajectoryPtPtr>& interpolated_traj)
{
  typedef std::vector<TrajectoryPtPtr> SegmentPoints;
  unsigned int point_count = 0;

  std::vector<double> segment_times;
  if(!computeFullSegmentTimes(coarse_traj,segment_times))
  {
    ROS_ERROR_STREAM("Failed to compute trajectory segment time values");
    return false;
  }


  // interpolating segments
  Eigen::Affine3d pose_start, pose_end;
  std::vector<double> seed(robot_model_->getDOF(),0.0f);
  SegmentPoints segment_points;
  std::vector<SegmentPoints> linear_segments(segment_times.size());
  std::vector<Eigen::Vector3d> segment_velocities(segment_times.size());
  double blend_segment_time = 2*std::abs(zone_radius_/tool_speed_);


  auto computeVel = [&](const descartes_core::TrajectoryPt& start_point,
      const descartes_core::TrajectoryPt& end_point,double duration) -> Eigen::Vector3d
    {
      Eigen::Affine3d start_pose, end_pose;
      Eigen::Vector3d start_position, end_position, vel;
      start_point.getNominalCartPose(seed,*robot_model_,start_pose);
      end_point.getNominalCartPose(seed,*robot_model_,end_pose);
      start_position = start_pose.translation();
      end_position = end_pose.translation();

      return (end_position - start_position)/ duration;
    };

  // computing linear segments
  for(std::size_t i = 0; i < segment_times.size();i++)
  {
    descartes_core::TrajectoryPtPtr ps = coarse_traj[i];
    descartes_core::TrajectoryPtPtr pe = coarse_traj[i+1];
    ps->getNominalCartPose(seed,*robot_model_,pose_start);
    pe->getNominalCartPose(seed,*robot_model_,pose_end);

    double start_time = (i == 0 ? 0.0f : 0.5f*blend_segment_time);
    double end_time = (i == segment_times.size() - 1) ? segment_times[i] : (segment_times[i] - 0.5f*blend_segment_time);
    LinearSegment lsegment = LinearSegment(pose_start,pose_end,segment_times[i],interpolation_interval_,
                                           std::make_pair(start_time,end_time));

    if(lsegment.interpolate(segment_points))
    {
      linear_segments[i] = segment_points;
      segment_velocities[i] = computeVel(*ps,*pe,segment_times[i]);
      point_count+=(segment_points.size() );
      segment_points.clear();


      ROS_DEBUG_STREAM("Linear segment contains "<<linear_segments[i].size()<<" points.");
    }
    else
    {
      ROS_ERROR_STREAM("Linear interpolation for segment "<<i<<" failed");
      return false;
    }
  }

  // computing blend segments
  std::vector<SegmentPoints> blend_segments(linear_segments.size()-1);

  for(std::size_t i = 1 ; i < linear_segments.size() ; i++)
  {
    SegmentPoints& seg_start = linear_segments[i-1];
    SegmentPoints& seg_end = linear_segments[i];

    descartes_core::TrajectoryPtPtr& ps = seg_start.back();
    descartes_core::TrajectoryPtPtr& pe = seg_end.front();

    // determining segment start and end poses
    ps->getNominalCartPose(seed,*robot_model_,pose_start);
    pe->getNominalCartPose(seed,*robot_model_,pose_end);

    BlendSegment bsegment = BlendSegment(pose_start,pose_end,
                                         segment_velocities[i-1],segment_velocities[i],
                                         interpolation_interval_);
    if(bsegment.interpolate(segment_points))
    {
      blend_segments[i - 1] = segment_points;
      point_count+=(segment_points.size() - 2 );
      segment_points.clear();

      ROS_DEBUG_STREAM("Blend segment contains "<<blend_segments[i - 1].size()<<" points.");
    }
    else
    {
      ROS_ERROR_STREAM("Blend interpolation failed between linear segments "<<i-1<<" to "<<i);
      return false;
    }
  }

  // concatenating segments
  interpolated_traj.clear();
  interpolated_traj.reserve(point_count);
  for(std::size_t i = 1; i < linear_segments.size();i++)
  {
    if(i == 1)
    {
      SegmentPoints& lseg0 = linear_segments[i-1];
      SegmentPoints& lsegf = linear_segments[i];
      SegmentPoints& bseg = blend_segments[i-1];

      interpolated_traj.insert(interpolated_traj.end(), lseg0.begin(),lseg0.end()-1);
      interpolated_traj.insert(interpolated_traj.end(), bseg.begin(),bseg.end()-1);
      interpolated_traj.insert(interpolated_traj.end(), lsegf.begin(),lsegf.end()-1);
    }
    else if(i == (linear_segments.size() - 1))
    {
      SegmentPoints& lseg = linear_segments[i];
      SegmentPoints& bseg = blend_segments[i-1];

      interpolated_traj.insert(interpolated_traj.end(), bseg.begin(),bseg.end()-1);
      interpolated_traj.insert(interpolated_traj.end(), lseg.begin(),lseg.end());
    }
    else
    {
      SegmentPoints& lseg = linear_segments[i];
      SegmentPoints& bseg = blend_segments[i-1];

      interpolated_traj.insert(interpolated_traj.end(), bseg.begin(),bseg.end()-1);
      interpolated_traj.insert(interpolated_traj.end(), lseg.begin(),lseg.end()-1);
    }
  }

  return true;
}

} /* namespace descartes_trajectory */
