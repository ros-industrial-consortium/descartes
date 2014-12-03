/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Dan Solomon
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
 * cart_trajectory_pt.cpp
 *
 *  Created on: Oct 3, 2014
 *      Author: Dan Solomon
 */

#include <console_bridge/console.h>
#include <ros/console.h>
#include "descartes_core/cart_trajectory_pt.h"

#define NOT_IMPLEMENTED_ERR(ret) logError("%s not implemented", __PRETTY_FUNCTION__); return ret;


namespace descartes_core
{
 EigenSTL::vector_Affine3d uniform(const TolerancedFrame & frame, const double orient_increment,
                                  const double pos_increment)
{

   EigenSTL::vector_Affine3d rtn;

   if(pos_increment < 0.0 || orient_increment < 0.0)
   {
     ROS_WARN_STREAM("Negative position/orientation intcrement: " << pos_increment
                     << "/" << orient_increment);
     rtn.clear();
     return rtn;
   }

  Eigen::Affine3d sampled_frame;

  // Calculating the number of samples for each tolerance (position and orientation)
  size_t ntx, nty, ntz, nrx, nry, nrz;

  if(orient_increment > 0)
  {
    nrx = (frame.orientation_tolerance.x_upper - frame.orientation_tolerance.x_lower)/orient_increment + 1;
    nry = (frame.orientation_tolerance.y_upper - frame.orientation_tolerance.y_lower)/orient_increment + 1;
    nrz = (frame.orientation_tolerance.z_upper - frame.orientation_tolerance.z_lower)/orient_increment + 1;
  }
  else
  {
    nrx = nry = nrz = 1;
  }

  if(pos_increment > 0)
  {
    ntx = (frame.position_tolerance.x_upper - frame.position_tolerance.x_lower)/pos_increment + 1;
    nty = (frame.position_tolerance.y_upper - frame.position_tolerance.y_lower)/pos_increment + 1;
    ntz = (frame.position_tolerance.z_upper - frame.position_tolerance.z_lower)/pos_increment + 1;
  }
  else
  {
    ntx = nty = ntz = 1;
  }

  // Estimating the number of samples base on tolerance zones and sampling increment.
  size_t est_num_samples = ntx * nty * ntz * nrx * nry * nrz;

  ROS_DEBUG_STREAM("Estimated number of samples: " << est_num_samples << ", reserving space");
  rtn.reserve(est_num_samples);

  //TODO: The following for loops do not ensure that the rull range is sample (lower to upper)
  //since there could be round off error in the incrementing of samples.  As a result, the
  //exact upper bound may not be sampled.  Since this isn't a final implementation, this will
  //be ignored.
  double rx, ry, rz, tx, ty, tz;

  for(size_t ii = 0; ii < nrx; ++ii)
  {
    rx = frame.orientation_tolerance.x_lower + orient_increment * ii;
    for(size_t jj = 0; jj < nry; ++jj)
    {
      ry = frame.orientation_tolerance.y_lower + orient_increment * jj;
      for(size_t kk = 0; kk < nrz; ++kk)
      {
        rz = frame.orientation_tolerance.z_lower + orient_increment * kk;
        for(size_t ll = 0; ll < ntx; ++ll)
        {
          tx = frame.position_tolerance.x_lower + pos_increment * ll;
          for(size_t mm = 0; mm < nty; ++mm)
          {
            ty = frame.position_tolerance.y_lower + pos_increment * mm;
            for(size_t nn = 0; nn < ntz; ++nn)
            {
              tz = frame.position_tolerance.z_lower + pos_increment * nn;
              sampled_frame = frame.frame*Eigen::Translation3d(tx,ty,tz) *
                  Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX());
              rtn.push_back(sampled_frame);
            }
          }
        }
      }
    }
  }
  ROS_DEBUG_STREAM("Uniform sampling of frame, utilizing orientation increment: " << orient_increment
                   << ", and position increment: " << pos_increment
                   << " resulted in " << rtn.size() << " samples");
  return rtn;
}

CartTrajectoryPt::CartTrajectoryPt():
  tool_base_(Eigen::Affine3d::Identity()),
  tool_pt_(Eigen::Affine3d::Identity()),
  wobj_base_(Eigen::Affine3d::Identity()),
  wobj_pt_(Eigen::Affine3d::Identity()),
  pos_increment_(0.0),
  orient_increment_(0.0)
{}

CartTrajectoryPt::CartTrajectoryPt(const Frame &wobj_base, const TolerancedFrame &wobj_pt, const Frame &tool,
                 const TolerancedFrame &tool_pt, double pos_increment, double orient_increment):
  tool_base_(tool),
  tool_pt_(tool_pt),
  wobj_base_(wobj_base),
  wobj_pt_(wobj_pt),
  pos_increment_(pos_increment),
  orient_increment_(orient_increment)
{}


CartTrajectoryPt::CartTrajectoryPt(const TolerancedFrame &wobj_pt, double pos_increment,
                                   double orient_increment):
  tool_base_(Eigen::Affine3d::Identity()),
  tool_pt_(Eigen::Affine3d::Identity()),
  wobj_base_(Eigen::Affine3d::Identity()),
  wobj_pt_(wobj_pt),
  pos_increment_(pos_increment),
  orient_increment_(orient_increment)
{}


CartTrajectoryPt::CartTrajectoryPt(const Frame &wobj_pt):
tool_base_(Eigen::Affine3d::Identity()),
tool_pt_(Eigen::Affine3d::Identity()),
wobj_base_(Eigen::Affine3d::Identity()),
wobj_pt_(wobj_pt),
pos_increment_(0),
orient_increment_(0)
{}

bool CartTrajectoryPt::getClosestCartPose(const std::vector<double> &seed_state,
                                          const RobotModel &model, Eigen::Affine3d &pose) const
{
  NOT_IMPLEMENTED_ERR(false);
}

bool CartTrajectoryPt::getNominalCartPose(const std::vector<double> &seed_state,
                                          const RobotModel &model, Eigen::Affine3d &pose) const
{
  /* Simply return wobj_pt expressed in world */
  pose = wobj_base_.frame * wobj_pt_.frame;
  return true;  //TODO can this ever return false?
}

void CartTrajectoryPt::getCartesianPoses(const RobotModel &model, EigenSTL::vector_Affine3d &poses) const
{
  poses.clear();
  EigenSTL::vector_Affine3d sampled_wobj_pts = uniform(wobj_pt_, orient_increment_,
                                                     pos_increment_);
  poses.reserve(sampled_wobj_pts.size());
  for(size_t wobj_pt = 0; wobj_pt < sampled_wobj_pts.size(); ++wobj_pt)
  {
    Eigen::Affine3d pose = wobj_base_.frame * sampled_wobj_pts[wobj_pt];
    if(model.isValid(pose))
    {
      poses.push_back(pose);
    }
  }
  if( poses.empty())
  {
    ROS_WARN("Failed for find ANY cartesian poses, returning");
  }
  else
  {
    ROS_DEBUG_STREAM("Get cartesian poses, sampled: " << sampled_wobj_pts.size()
                     << ", with " << poses.size() << " valid(returned) poses");
  }
}

bool CartTrajectoryPt::getClosestJointPose(const std::vector<double> &seed_state,
                                           const RobotModel &model,
                                           std::vector<double> &joint_pose) const
{
  NOT_IMPLEMENTED_ERR(false);
}

bool CartTrajectoryPt::getNominalJointPose(const std::vector<double> &seed_state,
                                           const RobotModel &model,
                                           std::vector<double> &joint_pose) const
{
  Eigen::Affine3d robot_pose = wobj_base_.frame * wobj_pt_.frame *
      tool_pt_.frame_inv * tool_base_.frame_inv;
  return model.getIK(robot_pose, seed_state, joint_pose);
}

void CartTrajectoryPt::getJointPoses(const RobotModel &model,
                                     std::vector<std::vector<double> > &joint_poses) const
{
  joint_poses.clear();
  EigenSTL::vector_Affine3d sampled_wobj_pts = uniform(wobj_pt_, orient_increment_,
                                                     pos_increment_);
  EigenSTL::vector_Affine3d sampled_tool_pts = uniform(tool_pt_, orient_increment_,
                                                       pos_increment_);
  size_t sample_size = sampled_wobj_pts.size() * sampled_tool_pts.size();
  joint_poses.reserve(sample_size);
  for(size_t wobj_pt = 0; wobj_pt < sampled_wobj_pts.size(); ++wobj_pt)
  {
    for(size_t tool_pt = 0; tool_pt < sampled_tool_pts.size(); ++tool_pt)
    {
      Eigen::Affine3d pose = wobj_base_.frame * sampled_wobj_pts[wobj_pt] *
          sampled_tool_pts[tool_pt].inverse() * tool_base_.frame_inv;
      std::vector<std::vector<double> > local_joint_poses;
      if(model.getAllIK(pose, local_joint_poses))
      {
        joint_poses.insert(joint_poses.end(), local_joint_poses.begin(), local_joint_poses.end());
      }
    }
  }
  if( joint_poses.empty())
  {
    ROS_WARN("Failed for find ANY joint poses, returning");
  }
  else
  {
    ROS_DEBUG_STREAM("Get joint poses, sampled: " << sample_size
                     << ", with " << joint_poses.size() << " valid(returned) poses");
  }
}

bool CartTrajectoryPt::isValid(const RobotModel &model) const
{
  Eigen::Affine3d robot_pose = wobj_base_.frame * wobj_pt_.frame *
      tool_pt_.frame_inv * tool_base_.frame_inv;
  return model.isValid(robot_pose);
}


bool CartTrajectoryPt::setDiscretization(const std::vector<double> &discretization)
{
  NOT_IMPLEMENTED_ERR(false);
}

} /* namespace descartes_core */
