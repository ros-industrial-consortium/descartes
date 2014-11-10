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
 * joint_trajectory_pt.h
 *
 *  Created on: Oct 3, 2014
 *      Author: Dan Solomon
 */

#ifndef JOINT_TRAJECTORY_PT_H_
#define JOINT_TRAJECTORY_PT_H_

#include <vector>
#include "descartes_core/trajectory_pt.h"

namespace descartes_core
{

//TODO add warning if non-zero tolerances are specified because initial implementation will only allow fixed joints
struct JointTolerance
{
  JointTolerance(): lower(0.), upper(0.) {};
  JointTolerance(double low, double high):
    lower(std::abs(low)), upper(std::abs(high)) {};

  double lower, upper;
};

struct TolerancedJointValue
{
  TolerancedJointValue() {};
  TolerancedJointValue(double _nominal, double _tol_above, double _tol_below):
    nominal(_nominal), tolerance(_tol_above, _tol_below) {};
  TolerancedJointValue(double _nominal)
  {
    *this = TolerancedJointValue(_nominal, 0., 0.);
  }

  double upperBound() const
  {
    return nominal+tolerance.upper;
  }

  double lowerBound() const
  {
    return nominal-tolerance.lower;
  }

  double range() const
  {
    return upperBound() - lowerBound();
  }

  double nominal;
  JointTolerance tolerance;
};

/**@brief Joint Trajectory Point used to describe a joint goal for a robot trajectory.
 *
 * Background:
 * The TOOL is something held by the robot. It is located relative to robot wrist/tool plate.
 * The WOBJ is something that exists in the world/global environment that is not held by robot.
 *
 * For a JointTrajectoryPt, the transform from wrist to tool, and base to workobject, are defined by fixed frames.
 * These transforms are important when calculating interpolation.
 * The joint position is specified as a nominal with upper/lower tolerances.
 *
 * The get*Pose() methods of JointTrajectoryPt try to set joint positions of a robot such that @e tool_ is coincident with @e wobj_.
 */
class JointTrajectoryPt: public TrajectoryPt
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;      //TODO is this needed when Frame already has it?

public:
  struct JointPointData
  {
    JointPointData():
      tool(Eigen::Affine3d::Identity()), wobj(Eigen::Affine3d::Identity())
    {};

    JointPointData(const std::vector<TolerancedJointValue> &_joints,
                   const Frame &_tool, const Frame &_wobj):
                     joint_position(_joints),
                     tool(_tool), wobj(_wobj)
    {};

    std::vector<TolerancedJointValue> joint_position;   /**<@brief Fixed joint position with tolerance */
    Frame tool;                                         /**<@brief Transform from robot wrist to active tool pt. */
    Frame wobj;                                         /**<@brief Transform from world to active workobject pt. */
  };

public:
  /**
    @brief Default joint trajectory point constructor.  All frames initialized to Identity, joint
    values left empty
    */
  JointTrajectoryPt();

  /**
    @brief Full joint trajectory point constructor
    @param joints Fixed joint position with tolerance
    @param tool Transform from robot wrist to active tool pt.
    @param wobj Transform from world to active workobject pt.
    */
  JointTrajectoryPt(const std::vector<TolerancedJointValue> &joints, const Frame &tool, const Frame &wobj);


  /**
    @brief Full joint trajectory point constructor
    @param joints Fixed joint position with tolerance
    */
  JointTrajectoryPt(const std::vector<TolerancedJointValue> &joints);


  /**
    @brief Full joint trajectory point constructor
    @param joints Fixed joint position
    */
  JointTrajectoryPt(const std::vector<double> &joints);

  virtual ~JointTrajectoryPt() {};

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

  virtual const void* getPointData() const;

  //TODO complete
  virtual void getJointPoses(const RobotModel &model,
                               std::vector<std::vector<double> > &joint_poses) const;
  /** @} (end section) */

  //TODO complete
  virtual bool isValid(const RobotModel &model) const;

inline
  void setJoints(const std::vector<TolerancedJointValue> &joints)
  {
    point_data_.joint_position = joints;
  }

  inline
  void setTool(const Frame &tool)
  {
    point_data_.tool = tool;
  }

  inline
  void setWobj(const Frame &wobj)
  {
    point_data_.wobj = wobj;
  }
  /**@} (end Setters section) */

  virtual
  bool setSampler(const TrajectoryPtSamplerPtr &sampler);

  virtual
  bool sample(size_t n);

protected:

  JointPointData point_data_;

};

} /* namespace descartes_core */



#endif /* JOINT_TRAJECTORY_PT_H_ */
