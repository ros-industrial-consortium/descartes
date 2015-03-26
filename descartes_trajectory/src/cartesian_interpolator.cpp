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
#include <boost/uuid/uuid_io.hpp>

#define NOT_IMPLEMENTED_ERR(ret) ROS_ERROR("%s not implemented", __PRETTY_FUNCTION__); return ret;

using namespace descartes_core;

namespace descartes_trajectory
{

CartesianInterpolator::CartesianInterpolator():
    tool_speed_(0),
    time_step_(0),
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
                                       double tool_speed,double time_step, double zone_radius)
{
  robot_model_ = robot_model;
  tool_speed_ = std::abs(tool_speed);
  time_step_= std::abs(time_step);
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
    LinearSegment lsegment = LinearSegment(pose_start,pose_end,segment_times[i],time_step_,
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
                                         time_step_);
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
