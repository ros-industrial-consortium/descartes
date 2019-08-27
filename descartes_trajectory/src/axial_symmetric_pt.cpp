/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2018, Southwest Research Institute
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
#include "descartes_trajectory/axial_symmetric_pt.h"

using descartes_trajectory::TolerancedFrame;
using descartes_trajectory::AxialSymmetricPt;
using namespace descartes_core::utils;

namespace descartes_trajectory
{

AxialSymmetricPt::AxialSymmetricPt(const descartes_core::TimingConstraint& timing) : CartTrajectoryPt(timing)
{
}

AxialSymmetricPt::AxialSymmetricPt(double x, double y, double z, double rx, double ry, double rz,
                                   double orient_increment, FreeAxis axis,
                                   const descartes_core::TimingConstraint& timing)
  : CartTrajectoryPt(TolerancedFrame(toFrame(x, y, z, rx, ry, rz)),
                     0.0,               // The position discretization
                     orient_increment,  // Orientation discretization (starting at -2Pi and marching to 2Pi)
                     timing)
  , axis_{axis}
{}

AxialSymmetricPt::AxialSymmetricPt(const Eigen::Affine3d& pose, double orient_increment, FreeAxis axis,
                                   const descartes_core::TimingConstraint& timing)
  : CartTrajectoryPt(TolerancedFrame(pose),
                     0.0,               // The position discretization
                     orient_increment,  // Orientation discretization (starting at -2Pi and marching to 2Pi)
                     timing)
  , axis_{axis}
{}

static void computePoses(const Eigen::Affine3d& origin, const AxialSymmetricPt::FreeAxis& axis,
                         const double orientation_step, EigenSTL::vector_Affine3d& out)
{
  Eigen::Vector3d rot_axis;
  switch (axis)
  {
  case AxialSymmetricPt::FreeAxis::X_AXIS:
    rot_axis = Eigen::Vector3d::UnitX();
    break;
  case AxialSymmetricPt::FreeAxis::Y_AXIS:
    rot_axis = Eigen::Vector3d::UnitY();
    break;
  case AxialSymmetricPt::FreeAxis::Z_AXIS:
    rot_axis = Eigen::Vector3d::UnitZ();
    break;
  }

  const static double lower_limit = -M_PI;
  const static double upper_limit = M_PI;

  out.clear();

  const auto n_elements = static_cast<std::size_t>((upper_limit - lower_limit) / orientation_step) + 1u;
  out.reserve(n_elements);

  double s = lower_limit;
  for (std::size_t i = 0; i < n_elements; ++i)
  {
    out.push_back(origin * Eigen::AngleAxisd(s, rot_axis));
    s += orientation_step;
  }
}

void AxialSymmetricPt::getJointPoses(const descartes_core::RobotModel &model,
                                     std::vector<std::vector<double>>& joint_poses) const
{
  joint_poses.clear();

  EigenSTL::vector_Affine3d poses;
  computePoses(this->wobj_pt_.frame, axis_, this->orient_increment_, poses);

  joint_poses.reserve(poses.size());
  for (const auto& pose : poses)
  {
    std::vector<std::vector<double>> sols;
    if (model.getAllIK(pose, sols))
    {
      joint_poses.insert(joint_poses.end(), std::make_move_iterator(sols.begin()),
                         std::make_move_iterator(sols.end()));
    }
  }
}

void AxialSymmetricPt::getCartesianPoses(const descartes_core::RobotModel &model,
                                         EigenSTL::vector_Affine3d &poses) const
{
  // Computes all of the candidate poses then filters them based on IK - not sure how I feel about this but I'll keep
  // the behaviour already present in getCartesianPoses
  poses.clear();
  EigenSTL::vector_Affine3d all_poses;
  computePoses(wobj_pt_.frame, axis_, orient_increment_, all_poses);

  poses.reserve(all_poses.size());
  for (const auto& pose : all_poses)
  {
    if (model.isValid(pose)) poses.push_back(pose);
  }
}

}  // end of namespace descartes_trajectory
