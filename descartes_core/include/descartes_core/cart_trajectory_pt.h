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

#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <ros/console.h>
#include "descartes_core/trajectory_pt.h"
#include "descartes_core/samplers/cart_pt_sampler_base.h"

typedef boost::shared_ptr<kinematic_constraints::PositionConstraint> PositionConstraintPtr;
typedef boost::shared_ptr<kinematic_constraints::OrientationConstraint> OrientationConstraintPtr;

namespace descartes_core
{

/**@brief Description of a per-cartesian-axis tolerance.  This tolerance is not meant
  to be used directly but rather used as a common base for positional/orientation
  tolerances.
 */
struct ToleranceBase
{

  /**
    @brief Returns a tolerance for points with symetric tolerance zones.
    @param x, y, z Nomial position
    @param x_tol, y_tol, z_tol Full tolerance zone (upper/lower values determined by nominal
            +/- tol/2.0
    */
  template<typename T>
  static T createSymmetric(double x, double y, double z,
                                          double x_tol, double y_tol, double z_tol)
  {
    double x_lower = x - x_tol/2.0;
    double x_upper = x + x_tol/2.0;

    double y_lower = y - y_tol/2.0;
    double y_upper = y + y_tol/2.0;

    double z_lower = z - z_tol/2.0;
    double z_upper = z + z_tol/2.0;

    return(T(x_lower, x_upper, y_lower, y_upper, z_lower, z_upper));
  }

  /**
    @brief Tolerance constructor for points with symetric tolerance zones. See ::init
    @param x, y, z Nominal position
    #param tol Total tolerance zone (assumed symetric about nominal)
    */
  template<typename T>
  static T createSymmetric(const double& x, const double& y, const double& z,
                                          const double& tol)
  {
    return(createSymmetric<T>(x, y, z, tol, tol, tol));
  }

  /**
    @brief Tolerance constructor for nomial points (no tolerance). See ::init
    @param x, y, z
    */
  template<typename T>
  static T zeroTolerance(const double& x, const double& y, const double& z)
  {
    return(createSymmetric<T>(x, y, z, 0.0, 0.0, 0.0));
  }


  /**
    @brief Default constructor, all upper/lower limits initialized to 0
    */
  ToleranceBase(): x_upper(0.), y_upper(0.), z_upper(0.),
    x_lower(0.), y_lower(0.), z_lower(0.)
  {}

  /**
    @brief Full construtor, all values set
    @param x/y/z_upper/lower_lim Upper/lower limits for each coordinate.
    */
  ToleranceBase(double x_lower_lim, double  x_upper_lim, double y_lower_lim, double y_upper_lim,
                double z_lower_lim, double z_upper_lim):
    x_upper(x_upper_lim), y_upper(y_upper_lim), z_upper(z_upper_lim),
    x_lower(x_lower_lim), y_lower(y_lower_lim), z_lower(z_lower_lim)
  {
    ROS_DEBUG_STREAM("Creating fully defined Tolerance(base type)");
    ROS_DEBUG_STREAM("Initializing x tolerance (lower/upper)" << x_lower << "/" << x_upper);
    ROS_DEBUG_STREAM("Initializing y tolerance (lower/upper)" << y_lower << "/" << y_upper);
    ROS_DEBUG_STREAM("Initializing z tolerance (lower/upper)" << z_lower << "/" << z_upper);
  }

  void clear() {x_upper = y_upper = z_upper = x_lower = y_lower = z_lower = 0.;}

  double x_upper, y_upper, z_upper, x_lower, y_lower, z_lower;

};


/**@brief Description of a per-cartesian-axis linear tolerance on position
 * Combined with PositionConstraint to fully define pt position.
 */
struct PositionTolerance : public ToleranceBase
{
  PositionTolerance() : ToleranceBase()
  {}

  PositionTolerance(double x_lower_lim, double  x_upper_lim, double y_lower_lim, double y_upper_lim,
                    double z_lower_lim, double z_upper_lim):
    ToleranceBase(x_lower_lim, x_upper_lim, y_lower_lim, y_upper_lim, z_lower_lim, z_upper_lim)
  {
    ROS_DEBUG_STREAM("Created fully defined Position Tolerance");
  }

};

/**@brief Description of a per-axis rotational tolerance on orientation
 * Combined with OrientationConstraint to fully define pt orientation.
 */
struct OrientationTolerance : public ToleranceBase
{
  OrientationTolerance() : ToleranceBase()
  {}

  OrientationTolerance(double x_lower_lim, double  x_upper_lim, double y_lower_lim, double y_upper_lim,
                    double z_lower_lim, double z_upper_lim):
    ToleranceBase(x_lower_lim, x_upper_lim, y_lower_lim, y_upper_lim, z_lower_lim, z_upper_lim)
  {
    ROS_DEBUG_STREAM("Created fully defined Orientation Tolerance");
  }
};

/**@brief TolerancedFrame extends frame to include tolerances and constraints on position and orientation.
 * Samplers that are called on this object should sample within tolerance, and check if result satisfies constraints.
 */
struct TolerancedFrame: public Frame
{
  TolerancedFrame(){};
  TolerancedFrame(const Eigen::Affine3d &a):
    Frame(a)
  {
    Eigen::Vector3d t = a.translation();
    Eigen::Matrix3d m = a.rotation();
    Eigen::Vector3d rxyz = m.eulerAngles(0,1,2);
    position_tolerance =  ToleranceBase::createSymmetric<PositionTolerance>(t(0),t(1),t(2),0);
    orientation_tolerance = ToleranceBase::createSymmetric<OrientationTolerance>(rxyz(0),rxyz(1),rxyz(2),0);
  };
  TolerancedFrame(const Frame &a):
    Frame(a)
  {
    Eigen::Vector3d t = a.frame.translation();
    Eigen::Matrix3d m = a.frame.rotation();
    Eigen::Vector3d rxyz = m.eulerAngles(0,1,2);
    position_tolerance =  ToleranceBase::createSymmetric<PositionTolerance>(t(0),t(1),t(2),0);
    orientation_tolerance = ToleranceBase::createSymmetric<OrientationTolerance>(rxyz(0),rxyz(1),rxyz(2),0);
  };

  TolerancedFrame(const Eigen::Affine3d &a, const PositionTolerance &pos_tol,
                  const OrientationTolerance &orient_tol) :
    Frame(a), position_tolerance(pos_tol), orientation_tolerance(orient_tol){}

  PositionTolerance             position_tolerance;
  OrientationTolerance          orientation_tolerance;
  PositionConstraintPtr         position_constraint;
  OrientationConstraintPtr      orientation_constraint;

  Eigen::Vector3d upperPositionBound() const
  {
    return frame.translation() + Eigen::Vector3d(position_tolerance.x_upper, position_tolerance.y_upper, position_tolerance.z_upper);
  }

  Eigen::Vector3d lowerPositionBound() const
  {
    return frame.translation() - Eigen::Vector3d(position_tolerance.x_lower, position_tolerance.y_lower, position_tolerance.z_lower);
  }

  Eigen::Vector3d upperRPYBound() const
  {
    Eigen::Vector3d rpy = frame.rotation().eulerAngles(2,1,0);
    return Eigen::Vector3d(rpy(2), rpy(1), rpy(0)) + Eigen::Vector3d(orientation_tolerance.x_upper, orientation_tolerance.y_upper, orientation_tolerance.z_upper);  //TODO check that this is correct
  }

  Eigen::Vector3d lowerRPYBound() const
  {
    Eigen::Vector3d rpy = frame.rotation().eulerAngles(2,1,0);
    return Eigen::Vector3d(rpy(2), rpy(1), rpy(0)) - Eigen::Vector3d(orientation_tolerance.x_lower, orientation_tolerance.y_lower, orientation_tolerance.z_lower);  //TODO check that this is correct
  }
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
    @param pos_increment Position increment used for sampling
    @param pos_increment Orientation increment used for sampling
    */
  CartTrajectoryPt(const Frame &wobj_base, const TolerancedFrame &wobj_pt, const Frame &tool_base,
                   const TolerancedFrame &tool_pt, double pos_increment, double orient_increment);

  /**
    @brief Partial constructor of cartesian trajectory point (all frames not specified by parameters
    are initialized to Identity).  This constructor should be utilized to specify the robot tip (toleranced)
    point relative to the robot base.
    @param wobj_pt Underconstrained transform from object base to goal point on object.
    @param pos_increment Position increment used for sampling
    @param pos_increment Orientation increment used for sampling
    */
  CartTrajectoryPt(const TolerancedFrame &wobj_pt, double pos_increment, double orient_increment);


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

  /** @name Sampling
   * @{
   */
  /**@brief Assign a Cartesian pt sampler for tool frame of CartTrajectoryPt (similar to @e setWobjSampler).
   *
   * If successful, @e tool_sampler_ will be assigned to @e sampler.
   *
   * @param[in] sampler Shared ptr to a CartPtSampler.
   * @return True if sampler is initialized with point data.
   */
  virtual
  bool setToolSampler(const CartPtSamplerBasePtr &sampler);
  virtual
  bool setWobjSampler(const CartPtSamplerBasePtr &sampler);


  /**@brief Get samples. This implementation is called by sampleTool() and sampleGoal().
   *
   * @param n Number of times to perform sampling. Actual number of returned samples may be different. A value of 0 means to get all available remaining samples.
   * @param sampler_ Sampler to use for sampling.
   * @return Set of Cartesian frame samples.
   */
  virtual
  EigenSTL::vector_Affine3d sample(size_t n, CartPtSamplerBasePtr &sampler_);
  virtual
  EigenSTL::vector_Affine3d sampleTool(size_t n);
  virtual
  EigenSTL::vector_Affine3d sampleWobj(size_t n);

  /** @} (end section) */

protected:
  Frame                         tool_base_;     /**<@brief Fixed transform from wrist/tool_plate to tool base. */
  TolerancedFrame               tool_pt_;       /**<@brief Underconstrained transform from tool_base to effective pt on tool. */
  Frame                         wobj_base_;     /**<@brief Fixed transform from WCS to base of object. */
  TolerancedFrame               wobj_pt_;       /**<@brief Underconstrained transform from object base to goal point on object. */

  double                        pos_increment_;    /**<@brief Sampling discretization in cartesian directions. */
  double                        orient_increment_; /**<@brief Sampling discretization in angular orientation. */

  CartPtSamplerBasePtr          tool_sampler_,  /**<@brief Cartesian sampler for tool frame. */
                                wobj_sampler_;  /**<@brief Cartesian sampler for wobj frame. */

};

} /* namespace descartes_core */



#endif /* CART_TRAJECTORY_PT_H_ */
