/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, ROS-Industrial Consortium
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
 * cart_trajectory_pt.h
 *
 *  Created on: Oct 3, 2014
 *      Author: dpsolomon
 */

#ifndef CART_TRAJECTORY_PT_H_
#define CART_TRAJECTORY_PT_H_

#include <descartes_trajectory_planning/trajectory_pt.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>

typedef boost::shared_ptr<kinematic_constraints::PositionConstraint> PositionConstraintPtr;
typedef boost::shared_ptr<kinematic_constraints::OrientationConstraint> OrientationConstraintPtr;


namespace descartes
{

/**@brief Description of a per-cartesian-axis linear tolerance on position
 * Combined with PositionConstraint to fully define pt position.
 */
struct PositionTolerance
{
  PositionTolerance(): x_upper(0.), y_upper(0.), z_upper(0.),
                       x_lower(0.), y_lower(0.), z_lower(0.)
  {}
  double x_upper, y_upper, z_upper, x_lower, y_lower, z_lower;

  void clear() {x_upper = y_upper = z_upper = x_lower = y_lower = z_lower = 0.;};
};

/**@brief Description of a per-axis rotational tolerance on orientation
 * Combined with OrientationConstraint to fully define pt orientation.
 */
struct OrientationTolerance
{
  OrientationTolerance(): x_upper(0.), y_upper(0.), z_upper(0.),
                          x_lower(0.), y_lower(0.), z_lower(0.)
  {}
  double x_upper, y_upper, z_upper, x_lower, y_lower, z_lower;

  void clear() {x_upper = y_upper = z_upper = x_lower = y_lower = z_lower = 0.;};
};

/**@brief TolerancedFrame extends frame to include tolerances and constraints on position and orientation.
 * Samplers that are called on this object should sample within tolerance, and check if result satisfies constraints.
 */
struct TolerancedFrame: public Frame
{
  PositionTolerance             position_tolerance;
  OrientationTolerance          orientation_tolerance;
  PositionConstraintPtr         position_constraint;
  OrientationConstraintPtr      orientation_constraint;
};

/**@brief Cartesian Trajectory Point used to describe a Cartesian goal for a robot trajectory.
 * (see TrajectoryPt class documentation for background on terms).
 * For CartTrajectoryPt, TOOL pose can be variable (e.g. robot holding workpiece) or fixed (e.g. robot holding MIG torch).
 * Similarly, the PART pose can be variable (e.g. robot riveting a workpiece) or fixed (e.g. stationary grinder that robot moves a tool against).
 * For a CartTrajectoryPt, tool pose is described by fixed transform from wrist to tool_base, and variable transform from tool_base to tool_point.
 * This allows the tolerances on tool pose to be easily expressed in a local tool frame.
 * Similarly, PART is located relative to world coordinate system, and is described by
 * fixed transform from world to part_base, and variable transform from part base to specific point on part.
 * Variable transforms of both TOOL and PART have tolerances on both position and orientation.
 * Optionally, additional constraints can be placed on position and orientation that can limit, but not expand, existing tolerances.
 */
class CartTrajectoryPt : public TrajectoryPt
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
public:
  CartTrajectoryPt();
  virtual ~CartTrajectoryPt();

protected:
  Frame                         tool_base_;             // Fixed transform from wrist/tool_plate to tool base.
  TolerancedFrame               tool_pt_;               // Underconstrained transform from tool_base to effective pt on tool.
  Frame                         object_base_;           // Fixed transform from WCS to base of object.
  TolerancedFrame               object_pt_;             // Underconstrained transform from object base to goal point on object. */
};

} /* namespace descartes */

#endif /* CART_TRAJECTORY_PT_H_ */
