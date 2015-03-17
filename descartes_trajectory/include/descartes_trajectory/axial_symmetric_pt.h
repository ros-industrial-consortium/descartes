/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2015, Southwest Research Institute
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

/**
 * @brief Specialization of cartesian trajectory point.
 *        Represents a cartesian pointwhose yaw (about z) is free.
 */
class AxialSymmetricPt : public descartes_trajectory::CartTrajectoryPt
{
public:
  /**
   * @brief This default constructor is exactly equivalent to CartTrajectoryPt's. Initializes all
   *        frames to identity.
   */
  AxialSymmetricPt() {}

  /**
   * @brief Constructs a cartesian trajectory point that places the robot tip at the position
   *        defined by the transform from the origin of the world. The transform is first a
   *        translation by the vector (x,y,z) followed by an XYZ rotation by (rx,ry,rz) respectively.
   *
   * @param x x component of translation part of transform
   * @param y y component of translation part of transform
   * @param z z component of translation part of transform
   * @param rx rotation about x axis
   * @param ry rotation about y axis
   * @param rz rotation about z axis (included only for nominal purposes)
   * @param orient_increment (in radians, discretization of space [-2Pi, 2Pi])
   */
  AxialSymmetricPt(double x, double y, double z, double rx, double ry, double rz, double orient_increment);

  virtual ~AxialSymmetricPt() {}
};


} // descartes trajectory

#endif // AXIAL_SYMMETRIC_PT_H
