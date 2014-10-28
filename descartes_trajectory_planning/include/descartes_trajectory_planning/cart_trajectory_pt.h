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
 * cart_trajectory_pt.h
 *
 *  Created on: Oct 3, 2014
 *      Author: Dan Solomon
 */

#ifndef CART_TRAJECTORY_PT_H_
#define CART_TRAJECTORY_PT_H_

#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include "descartes_trajectory_planning/trajectory_pt.h"

typedef boost::shared_ptr<kinematic_constraints::PositionConstraint> PositionConstraintPtr;
typedef boost::shared_ptr<kinematic_constraints::OrientationConstraint> OrientationConstraintPtr;

namespace descartes_core
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
  TolerancedFrame(){};
  TolerancedFrame(const Eigen::Affine3d &a):
    Frame(a) {};
  TolerancedFrame(const Frame &a):
    Frame(a) {};

  PositionTolerance             position_tolerance;
  OrientationTolerance          orientation_tolerance;
  PositionConstraintPtr         position_constraint;
  OrientationConstraintPtr      orientation_constraint;
};


/**@brief Cartesian Trajectory Point used to describe a Cartesian goal for a robot trajectory.
 *
 * Background:
 * For a general robotic process, TOOL pose can be variable (e.g. robot holding workpiece) or fixed (e.g. robot holding MIG torch).
 * Similarly, the WORKOBJECT pose can be variable (e.g. robot riveting a workpiece) or fixed (e.g. stationary grinder that robot moves a tool against).
 *
 * For a CartTrajectoryPt, TOOL pose is described by fixed transform from wrist to TOOL_BASE, and variable transform from TOOL_BASE to TOOL_PT.
 * This allows the tolerances on tool pose to be easily expressed in a local tool frame.
 * Similarly, WOBJ is located relative to world coordinate system, and is described by
 * fixed transform from world to WOBJ_BASE, and variable transform from WOBJ_BASE to specific point on part (WOBJ_PT).
 * Variable transforms of both TOOL and WOBJ have tolerances on both position and orientation.
 * Optionally, additional constraints can be placed on position and orientation that can limit, but not expand, existing tolerances.
 *
 * The get*Pose methods of CartTrajectoryPt try to set joint positions of a robot such that @e tool_pt_ is coincident with @e wobj_pt_.
 */
class CartTrajectoryPt : public TrajectoryPt
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
public:

  /**
    @brief Default cartesian trajectory point constructor.  All frames initialized to Identity
    */
  CartTrajectoryPt();

  /**
    @brief Full constructor of cartesian trajectory point
    @param wobj_base Fixed transform from WCS to base of object
    @param wobj_pt Underconstrained transform from object base to goal point on object.
    @param tool_base Fixed transform from wrist/tool_plate to tool base
    @param tool_pt Underconstrained transform from tool_base to effective pt on tool.
    */
  CartTrajectoryPt(const Frame &wobj_base, const TolerancedFrame &wobj_pt, const Frame &tool_base,
                   const TolerancedFrame &tool_pt);


  /**
    @brief Partial constructor of cartesian trajectory point (all frames not specified by parameters
    are initialized to Identity).  This constructor should be utilized to specify the robot tip (toleranced)
    point relative to the robot base.
    @param wobj_pt Underconstrained transform from object base to goal point on object.
    */
  CartTrajectoryPt(const TolerancedFrame &wobj_pt);


  /**
    @brief Partial constructor of cartesian trajectory point (all frames not specified by parameters
    are initialized to Identity).  This constructor should be utilized to specify the robot tip (NOT toleranced)
    point relative to the robot base.
    @param wobj_pt Underconstrained transform from object base to goal point on object.
    */
  CartTrajectoryPt(const Frame &wobj_pt);


  virtual ~CartTrajectoryPt() {};


  /**@name Getters for Cartesian pose(s)
     * @{
     */

    //TODO complete
    virtual bool getClosestCartPose(const std::vector<double> &seed_state,
                                      const RobotModel &model, Eigen::Affine3d &pose) const;

    //TODO complete
    virtual bool getNominalCartPose(const std::vector<double> &seed_state,
                                      const RobotModel &model, Eigen::Affine3d &pose) const;

    //TODO complete
    virtual void getCartesianPoses(const RobotModel &model, EigenSTL::vector_Affine3d &poses) const;
    /** @} (end section) */

    /**@name Getters for joint pose(s)
     * @{
     */

    //TODO complete
    virtual bool getClosestJointPose(const std::vector<double> &seed_state,
                                       const RobotModel &model,
                                       std::vector<double> &joint_pose) const;
    //TODO complete
    virtual bool getNominalJointPose(const std::vector<double> &seed_state,
                                       const RobotModel &model,
                                       std::vector<double> &joint_pose) const;

    //TODO complete
    virtual void getJointPoses(const RobotModel &model,
                                 std::vector<std::vector<double> > &joint_poses) const;
    /** @} (end section) */

    //TODO complete
    virtual bool isValid(const RobotModel &model) const;
  /**@brief Set discretization. Cartesian points can have position and angular discretization.
   * @param discretization Vector of discretization values. Must be length 2 or 6 (position/orientation or separate xyzrpy).
   * @return True if vector is valid length/values. TODO what are valid values?
   */
  virtual bool setDiscretization(const std::vector<double> &discretization);

  inline
  void setTool(const Frame &base, const TolerancedFrame &pt)
  {
    tool_base_ = base;
    tool_pt_ = pt;
  }

  inline
  void setWobj(const Frame &base, const TolerancedFrame &pt)
  {
    wobj_base_ = base;
    wobj_pt_ = pt;
  }


protected:
  Frame                         tool_base_;     /**<@brief Fixed transform from wrist/tool_plate to tool base. */
  TolerancedFrame               tool_pt_;       /**<@brief Underconstrained transform from tool_base to effective pt on tool. */
  Frame                         wobj_base_;     /**<@brief Fixed transform from WCS to base of object. */
  TolerancedFrame               wobj_pt_;       /**<@brief Underconstrained transform from object base to goal point on object. */


};

} /* namespace descartes_core */
// For backwards namespace compatability
namespace descartes = descartes_core;

#endif /* CART_TRAJECTORY_PT_H_ */
