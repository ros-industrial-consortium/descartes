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

#include "descartes_core_test/cartesian_robot.h"
#include "descartes_core/pretty_print.hpp"
#include "eigen_conversions/eigen_kdl.h"
#include "ros/console.h"

namespace descartes_core_test
{


  bool CartesianRobot::getIK(const Eigen::Affine3d &pose, const std::vector<double> &seed_state,
                     std::vector<double> &joint_pose) const
  {
    bool rtn = false;
    KDL::Frame kdl_frame;
    tf::transformEigenToKDL(pose, kdl_frame);

    joint_pose.resize(6, 0.0);
    joint_pose[0] = kdl_frame.p.x();
    joint_pose[1] = kdl_frame.p.y();
    joint_pose[2] = kdl_frame.p.z();
    kdl_frame.M.GetRPY(joint_pose[3], joint_pose[4], joint_pose[5]);

    ROS_DEBUG_STREAM("IK result for pose: " << pose.matrix() << " is " << joint_pose);

    if(isValid(joint_pose))
    {
      rtn = true;
    }
    else
    {
      rtn = false;
      ROS_WARN("IK calculated pose, not valid");
    }
    return rtn;
  }



  bool CartesianRobot::getAllIK(const Eigen::Affine3d &pose,
                                std::vector<std::vector<double> > &joint_poses) const
  {
    std::vector<double>empty;
    joint_poses.resize(1);
    return getIK(pose, empty, joint_poses[0]);
  }



  bool CartesianRobot::getFK(const std::vector<double> &joint_pose, Eigen::Affine3d &pose) const
  {
    bool rtn = false;

    if(isValid(joint_pose))
    {
      pose = Eigen::Translation3d(joint_pose[0], joint_pose[1], joint_pose[2]) *
          Eigen::AngleAxisd(joint_pose[5], Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(joint_pose[4], Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(joint_pose[3], Eigen::Vector3d::UnitX());
      rtn = true;
    }
    else
    {
      ROS_WARN_STREAM("Invalid joint pose passed to get FK, joint pose" << joint_pose);
      rtn = false;
    }

    return rtn;
  }



  bool CartesianRobot::isValid(const std::vector<double> &joint_pose) const
  {
    bool rtn = false;

    rtn = ( fabs(joint_pose[0]) <= pos_limit_ &&
            fabs(joint_pose[1]) <= pos_limit_ &&
            fabs(joint_pose[3]) <= pos_limit_ &&
            fabs(joint_pose[4]) <= orient_limit_ &&
            fabs(joint_pose[5]) <= orient_limit_ &&
            fabs(joint_pose[6]) <= orient_limit_ );
    if(rtn)
    {
      ROS_DEBUG_STREAM("Joint pose: " << joint_pose << " valid");
    }
    else
    {
      ROS_DEBUG_STREAM("Joint pose: " << joint_pose << " NOT valid");
    }

    return rtn;
  }

  bool CartesianRobot::isValid(const Eigen::Affine3d &pose) const
  {
    bool rtn = false;
    double R, P, Y;
    KDL::Frame kdl_frame;
    tf::transformEigenToKDL(pose, kdl_frame);
    kdl_frame.M.GetRPY(R, P, Y);

    rtn = ( fabs(kdl_frame.p.x()) <= pos_limit_ &&
            fabs(kdl_frame.p.y()) <= pos_limit_ &&
            fabs(kdl_frame.p.z()) <= pos_limit_ &&
            fabs(R) <= orient_limit_ &&
            fabs(P) <= orient_limit_ &&
            fabs(Y) <= orient_limit_ );

    if(rtn)
    {
      ROS_DEBUG_STREAM("Cart pose: " << pose.matrix() << " valid");
    }
    else
    {
      ROS_DEBUG_STREAM("Cart pose: " << pose.matrix() << " NOT valid");
    }

    return rtn;
  }

} //descartes_core_test

