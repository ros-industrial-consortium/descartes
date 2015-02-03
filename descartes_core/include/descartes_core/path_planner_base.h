/*
 * path_planner_base.h
 *
 *  Created on: Jan 19, 2015
 *      Author: ros developer 
 */

#ifndef DESCARTES_CORE_PATH_PLANNER_BASE_H_
#define DESCARTES_CORE_PATH_PLANNER_BASE_H_

#include <descartes_core/trajectory_pt.h>
#include <descartes_core/robot_model.h>
#include <vector>

namespace descartes_core
{

class PathPlannerBase
{
public:
  virtual ~PathPlannerBase(){}

  /**
   * @brief Plans a path for the given robot model through the points in the input trajectory.
   * @param model robot model implementation for which to plan a path
   * @param sampling This value will be interpreted differently by each planner specialization.
   */
  virtual bool initialize(RobotModelConstPtr &model, double sampling = 0.1f) = 0;

  /**
   * @brief Generates a robot path from the trajectory.
   * @param traj the points used to plan the robot path
   */
  virtual bool planPath(const std::vector<TrajectoryPtPtr>& traj) = 0;

  /**
   * @brief Returns the robot path generated from the input trajectory
   * @param path Array of path points
   */
  virtual bool getPath(std::vector<TrajectoryPtPtr>& path) = 0;

  virtual bool addAfter(const TrajectoryPt::ID& ref_id,TrajectoryPtPtr tp) = 0;
  virtual bool addBefore(const TrajectoryPt::ID& ref_id,TrajectoryPtPtr tp) = 0;
  virtual bool remove(const TrajectoryPt::ID& ref_id) = 0;
  virtual bool modify(const TrajectoryPt::ID& ref_id,TrajectoryPtPtr tp) = 0;

protected:
  PathPlannerBase(){}
};

}

#endif /* DESCARTES_CORE_PATH_PLANNER_BASE_H_ */
