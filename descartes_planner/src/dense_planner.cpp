/*
 * dense_planner.cpp
 *
 *  Created on: Feb 9, 2015
 *      Author: ros developer
 */

#include <descartes_planner/dense_planner.h>
#include <boost/make_shared.hpp>

namespace descartes_planner
{
using namespace descartes_core;
DensePlanner::DensePlanner() : planning_graph_(), error_code_(descartes_core::PlannerError::UNINITIALIZED)
{
  error_map_ = { { PlannerError::OK, "OK" },
                 { PlannerError::EMPTY_PATH, "No path plan has been generated" },
                 { PlannerError::INVALID_ID, "ID is nil or isn't part of the path" },
                 { PlannerError::IK_NOT_AVAILABLE, "One or more ik solutions could not be found" },
                 { PlannerError::UNINITIALIZED, "Planner has not been initialized with a robot model" },
                 { PlannerError::INCOMPLETE_PATH, "Input trajectory and output path point cound differ" } };
}

DensePlanner::~DensePlanner()
{
}

bool DensePlanner::initialize(descartes_core::RobotModelConstPtr model)
{
  planning_graph_ =
      boost::shared_ptr<descartes_planner::PlanningGraph>(new descartes_planner::PlanningGraph(std::move(model)));
  error_code_ = descartes_core::PlannerErrors::EMPTY_PATH;
  return true;
}

bool DensePlanner::initialize(descartes_core::RobotModelConstPtr model,
                              descartes_planner::CostFunction cost_function_callback)
{
  planning_graph_ = boost::shared_ptr<descartes_planner::PlanningGraph>(
      new descartes_planner::PlanningGraph(std::move(model), cost_function_callback));
  error_code_ = descartes_core::PlannerErrors::EMPTY_PATH;
  return true;
}

bool DensePlanner::setConfig(const descartes_core::PlannerConfig& config)
{
  config_ = config;
  config_.clear();
  return true;
}

void DensePlanner::getConfig(descartes_core::PlannerConfig& config) const
{
  config = config_;
}

descartes_core::TrajectoryPt::ID DensePlanner::getPrevious(const descartes_core::TrajectoryPt::ID& ref_id)
{
  descartes_core::TrajectoryPt::ID id;
  auto predicate = [&ref_id](descartes_core::TrajectoryPtPtr p)
  {
    return ref_id == p->getID();
  };

  auto pos = std::find_if(path_.begin()++, path_.end(), predicate);
  if (pos == path_.end())
  {
    id = descartes_core::TrajectoryID::make_nil();
  }
  else
  {
    pos--;
    id = (*pos)->getID();
  }

  return id;
}

bool DensePlanner::updatePath()
{
  double c;
  std::list<descartes_trajectory::JointTrajectoryPt> list;
  if (planning_graph_->getShortestPath(c, list))
  {
    error_code_ = descartes_core::PlannerErrors::OK;
    for (auto&& p : list)
    {
      path_.push_back(boost::make_shared<descartes_trajectory::JointTrajectoryPt>(std::move(p)));
    }
    return true;
  }
  else
  {
    error_code_ = descartes_core::PlannerErrors::UKNOWN;
    return false;
  }
}

descartes_core::TrajectoryPt::ID DensePlanner::getNext(const descartes_core::TrajectoryPt::ID& ref_id)
{
  descartes_core::TrajectoryPt::ID id;
  auto predicate = [&ref_id](descartes_core::TrajectoryPtPtr p)
  {
    return ref_id == p->getID();
  };

  auto pos = std::find_if(path_.begin(), path_.end() - 2, predicate);
  if (pos == path_.end())
  {
    id = descartes_core::TrajectoryID::make_nil();
  }
  else
  {
    pos++;
    id = (*pos)->getID();
  }
  return id;
}

descartes_core::TrajectoryPtPtr DensePlanner::get(const descartes_core::TrajectoryPt::ID& ref_id)
{
  descartes_core::TrajectoryPtPtr p;
  auto predicate = [&ref_id](descartes_core::TrajectoryPtPtr p)
  {
    return ref_id == p->getID();
  };

  auto pos = std::find_if(path_.begin(), path_.end() - 2, predicate);
  if (pos == path_.end())
  {
    p.reset();
  }
  else
  {
    p = *pos;
  }
  return p;
}

bool DensePlanner::planPath(const std::vector<descartes_core::TrajectoryPtPtr>& traj)
{
  if (error_code_ == descartes_core::PlannerError::UNINITIALIZED)
  {
    ROS_ERROR_STREAM("Planner has not been initialized");
    return false;
  }

  path_.clear();
  error_code_ = descartes_core::PlannerError::EMPTY_PATH;

  if (planning_graph_->insertGraph(traj))
  {
    updatePath();
  }
  else
  {
    error_code_ = descartes_core::PlannerError::IK_NOT_AVAILABLE;
  }

  return descartes_core::PlannerError::OK == error_code_;
}

bool DensePlanner::getPath(std::vector<descartes_core::TrajectoryPtPtr>& path) const
{
  if (path_.empty())
    return false;

  path.assign(path_.begin(), path_.end());
  return error_code_ == descartes_core::PlannerError::OK;
}

bool DensePlanner::addAfter(const descartes_core::TrajectoryPt::ID& ref_id, descartes_core::TrajectoryPtPtr tp)
{
  if (path_.empty())
  {
    return false;
  }

  descartes_core::TrajectoryPt::ID next_id = getNext(ref_id);
  if (!next_id.is_nil())
  {
    if (planning_graph_->addTrajectory(tp, ref_id, next_id))
    {
      if (updatePath())
      {
        error_code_ = descartes_core::PlannerError::OK;
      }
      else
      {
        return false;
      }
    }
    else
    {
      error_code_ = descartes_core::PlannerErrors::IK_NOT_AVAILABLE;
      return false;
    }
  }
  else
  {
    error_code_ = descartes_core::PlannerError::INVALID_ID;
    return false;
  }

  return true;
}

bool DensePlanner::addBefore(const descartes_core::TrajectoryPt::ID& ref_id, descartes_core::TrajectoryPtPtr tp)
{
  if (path_.empty())
  {
    return false;
  }

  descartes_core::TrajectoryPt::ID prev_id = getPrevious(ref_id);
  if (!prev_id.is_nil())
  {
    if (planning_graph_->addTrajectory(tp, prev_id, ref_id))
    {
      if (updatePath())
      {
        error_code_ = descartes_core::PlannerError::OK;
      }
      else
      {
        return false;
      }
    }
    else
    {
      error_code_ = descartes_core::PlannerErrors::IK_NOT_AVAILABLE;
      return false;
    }
  }
  else
  {
    error_code_ = descartes_core::PlannerError::INVALID_ID;
    return false;
  }

  return true;
}

bool DensePlanner::remove(const descartes_core::TrajectoryPt::ID& ref_id)
{
  if (path_.empty())
  {
    return false;
  }

  descartes_core::TrajectoryPtPtr tp = get(ref_id);
  if (tp)
  {
    tp->setID(ref_id);
    if (planning_graph_->removeTrajectory(tp->getID())) // TODO: Clean up this extra copy & lookup
    {
      if (updatePath()) // TODO: Should we force an update here? What if the user wants to remove several points?
      {
        error_code_ = descartes_core::PlannerError::OK;
      }
      else
      {
        return false;
      }
    }
    else
    {
      error_code_ = descartes_core::PlannerErrors::IK_NOT_AVAILABLE;
      return false;
    }
  }
  else
  {
    error_code_ = descartes_core::PlannerError::INVALID_ID;
    return false;
  }

  return true;
}

bool DensePlanner::modify(const descartes_core::TrajectoryPt::ID& ref_id, descartes_core::TrajectoryPtPtr tp)
{
  if (path_.empty())
  {
    return false;
  }

  if (!ref_id.is_nil())
  {
    tp->setID(ref_id);
    if (planning_graph_->modifyTrajectory(tp))
    {
      if (updatePath())
      {
        error_code_ = descartes_core::PlannerError::OK;
      }
      else
      {
        return false;
      }
    }
    else
    {
      error_code_ = descartes_core::PlannerErrors::IK_NOT_AVAILABLE;
      return false;
    }
  }
  else
  {
    error_code_ = descartes_core::PlannerError::INVALID_ID;
    return false;
  }

  return true;
}

int DensePlanner::getErrorCode() const
{
  return error_code_;
}

bool DensePlanner::getErrorMessage(int error_code, std::string& msg) const
{
  std::map<int, std::string>::const_iterator it = error_map_.find(error_code);

  if (it != error_map_.cend())
  {
    msg = it->second;
  }
  else
  {
    return false;
  }
  return true;
}

} /* namespace descartes_core */
