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

#ifndef AXIAL_SYMMETRIC_PT_H
#define AXIAL_SYMMETRIC_PT_H

#include <descartes_trajectory/cart_trajectory_pt.h>

namespace descartes_trajectory
{

/**@brief Specialization of cartesian trajectory point.  Represents a cartesian point
  whose yaw (about z) is free.
 */
class AxialSymmetricPt : public descartes_trajectory::CartTrajectoryPt
{
public:

  AxialSymmetricPt(){};

  AxialSymmetricPt(double x, double y, double z, double rx, double ry, double rz, double orient_increment) :
    CartTrajectoryPt(descartes_trajectory::TolerancedFrame(
                       descartes_trajectory::utils::toFrame(x, y, z, rx, ry, rz, descartes_trajectory::utils::EulerConventions::XYZ),
                       descartes_trajectory::ToleranceBase::zeroTolerance<descartes_trajectory::PositionTolerance>
                       (x, y, z),
                       descartes_trajectory::ToleranceBase::createSymmetric<descartes_trajectory::OrientationTolerance>
                       (rx, ry, 0, 0, 0, 2.0 * M_PI)),
                     0.0, orient_increment)
  {
  }

  virtual ~AxialSymmetricPt() {};
};


} // descartes trajectory

#endif // AXIAL_SYMMETRIC_PT_H
