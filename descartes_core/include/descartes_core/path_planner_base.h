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

  virtual bool initialize(RobotModelConstPtr &model,const std::vector<TrajectoryPtPtr>& traj,
            double sampling = 0.1f) = 0;

  virtual bool addAfter(const TrajectoryPt::ID& ref_id,TrajectoryPtPtr cp) = 0;
  virtual bool addBefore(const TrajectoryPt::ID& ref_id,TrajectoryPtPtr cp) = 0;
  virtual bool remove(const TrajectoryPt::ID& ref_id) = 0;
  virtual bool modify(const TrajectoryPt::ID& ref_id,TrajectoryPtPtr cp) = 0;
  virtual bool get(const CartTrajectoryPt::ID& cart_id,TrajectoryPtPtr& j);
  virtual bool get(int index, TrajectoryPtPtr& j = 0);
  virtual int size() = 0;

protected:
  PathPlannerBase(){}
};

}

#endif /* DESCARTES_CORE_PATH_PLANNER_BASE_H_ */
