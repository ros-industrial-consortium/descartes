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
/*
 * planning_graph.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: Dan Solomon
 *      Author: Jonathan Meyer
 */

#include "descartes_planner/planning_graph.h"
#include "descartes_planner/ladder_graph_dag_search.h"
#include "descartes_planner/planning_graph_edge_policy.h"
#include <ros/console.h>

using namespace descartes_core;
using namespace descartes_trajectory;

namespace descartes_planner
{

PlanningGraph::PlanningGraph(RobotModelConstPtr model, CostFunction cost_function_callback)
  : graph_(model->getDOF()), robot_model_(std::move(model)), custom_cost_function_(cost_function_callback)
{}

bool PlanningGraph::insertGraph(const std::vector<TrajectoryPtPtr>& points)
{
  if (points.size() < 2)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": must provide at least 2 input trajectory points.");
    return false;
  }

  if (graph_.size() > 0) clear();

  // generate solutions for this point
  std::vector<std::vector<std::vector<double>>> all_joint_sols;
  if (!calculateJointSolutions(points.data(), points.size(), all_joint_sols))
  {
    return false;
  }

  // insert into graph as vertices
  graph_.resize(points.size());
  for (std::size_t i = 0; i < points.size(); ++i)
  {
    graph_.assignRung(i, points[i]->getID(), points[i]->getTiming(), all_joint_sols[i]);
  }

  // now we have a graph with data in the 'rungs' and we need to compute the edges
  #pragma omp parallel for
  for (std::size_t i = 0; i < graph_.size() - 1; ++i)
  {
    computeAndAssignEdges(i, i + 1);
  }

  return true;
}

bool PlanningGraph::addTrajectory(TrajectoryPtPtr point, TrajectoryPt::ID previous_id, TrajectoryPt::ID next_id)
{
  assert(previous_id.is_nil() || graph_.indexOf(previous_id).second); // These tests ensure that the user points
  assert(next_id.is_nil() || graph_.indexOf(next_id).second);         // actually exist. Maybe should be part of
                                                                       // the normal interface? TODO
  auto ns = graph_.indexOf(next_id);

  // Next & prev can be 'null' indicating end & start of trajectory
  std::vector<std::vector<std::vector<double>>> poses;
  calculateJointSolutions(&point, 1, poses); // TODO: If there are no points, return false?

  // Insert new point into graph
  auto insert_idx = ns.second ? ns.first : graph_.size();
  graph_.insertRung(insert_idx);
  graph_.assignRung(insert_idx, point->getID(), point->getTiming(), poses[0]);

  // Build edges from prev point, if applicable
  if (!previous_id.is_nil())
  {
    auto prev_idx = insert_idx - 1;
    computeAndAssignEdges(prev_idx, insert_idx);
  }

  // Build edges to next point, if applicable
  if (!next_id.is_nil())
  {
    auto next_idx = insert_idx + 1;
    computeAndAssignEdges(insert_idx, next_idx);
  }

  return true;
}

bool PlanningGraph::modifyTrajectory(TrajectoryPtPtr point)
{
  auto s = graph_.indexOf(point->getID());
  if (!s.second) return false; // no such point
  auto idx = s.first;

  // we will need to recompute some vertices now
  std::vector<std::vector<std::vector<double>>> poses;
  calculateJointSolutions(&point, 1, poses); // TODO: If there are no points, return false?

  // clear vertices & edges of 'point'
  graph_.clearVertices(idx);
  graph_.clearEdges(idx);
  graph_.assignRung(idx, point->getID(), point->getTiming(), poses[0]);

  // If there is a previous point, compute new edges
  if (!graph_.isFirst(idx))
  {
    auto prev_idx = idx - 1;
    computeAndAssignEdges(prev_idx, idx);
  }

  // If there is a next point, compute new edges
  if (!graph_.isLast(idx))
  {
    auto next_idx = idx + 1;
    computeAndAssignEdges(idx, next_idx);
  }

  return true;
}

bool PlanningGraph::removeTrajectory(const TrajectoryPt::ID& point)
{
  // Remove a point from the graph
  auto s = graph_.indexOf(point);
  if (!s.second) return false;

  auto in_middle = !graph_.isFirst(s.first) && !graph_.isLast(s.first);

  // remove the vertices & edges associated with this point
  graph_.removeRung(s.first);

  // recompute edges from previous rung to next rung, if applicable
  if (in_middle)
  {
    auto prev_idx = s.first - 1;
    auto next_idx = s.first; // We erased a point, so the indexes have collapsed by one
    computeAndAssignEdges(prev_idx, next_idx);
  }

  return true;
}

bool PlanningGraph::getShortestPath(double& cost, std::list<JointTrajectoryPt>& path)
{
  DAGSearch search (graph_);
  cost = search.run();
  if (cost == std::numeric_limits<double>::max()) return false;

  auto path_idxs = search.shortestPath();
  const auto dof = graph_.dof();

  for (size_t i = 0; i < path_idxs.size(); ++i)
  {
    const auto idx = path_idxs[i];
    const auto* data = graph_.vertex(i, idx);
    const auto& tm = graph_.getRung(i).timing;

    auto pt = JointTrajectoryPt(std::vector<double>(data, data + dof), tm);
    path.push_back(std::move(pt));
  }

  ROS_INFO("Computed path of length %lu with cost %lf", path_idxs.size(), cost);

  return true;
}

bool PlanningGraph::calculateJointSolutions(const TrajectoryPtPtr* points, const std::size_t count,
                                            std::vector<std::vector<std::vector<double>>>& poses) const
{
  poses.resize(count);

  #pragma omp parallel for
  for (std::size_t i = 0; i < count; ++i)
  {
    std::vector<std::vector<double>> joint_poses;
    points[i]->getJointPoses(*robot_model_, joint_poses);

    if (joint_poses.empty())
    {
      ROS_ERROR_STREAM(__FUNCTION__ << ": IK failed for input trajectory point with ID = " << points[i]->getID());
//     return false;
    }

    poses[i] = std::move(joint_poses);
  }

  for (const auto& sols : poses)
  {
    if (sols.empty())
      return false;
  }

  return true;
}

void PlanningGraph::computeAndAssignEdges(const std::size_t start_idx, const std::size_t end_idx)
{
  assert(end_idx > start_idx);
  assert(end_idx - start_idx == 1);

  const auto& joints1 = graph_.getRung(start_idx).data;
  const auto& joints2 = graph_.getRung(end_idx).data;
  const auto& tm = graph_.getRung(end_idx).timing;
  const auto dof = robot_model_->getDOF();

  const auto start_size = joints1.size() / dof;
  const auto end_size = joints2.size() / dof;

  bool b;
  std::vector<LadderGraph::EdgeList> edges;

  if (!custom_cost_function_ && tm.isSpecified())
  {
    DefaultEdgesWithTime builder (start_size, end_size, dof, tm.upper, robot_model_->getJointVelocityLimits());
    edges = calculateEdgeWeights(builder, joints1, joints2, dof, b);
  }
  else if (custom_cost_function_ && tm.isSpecified())
  {
    CustomEdgesWithTime builder (start_size, end_size, dof, tm.upper, robot_model_->getJointVelocityLimits(), custom_cost_function_);
    edges = calculateEdgeWeights(builder, joints1, joints2, dof, b);
  }
  else if (!custom_cost_function_ && !tm.isSpecified())
  {
    DefaultEdgesWithoutTime builder (start_size, end_size, dof);
    edges = calculateEdgeWeights(builder, joints1, joints2, dof, b);
  }
  else
  {
    CustomEdgesWithoutTime builder (start_size, end_size, dof, custom_cost_function_);
    edges = calculateEdgeWeights(builder, joints1, joints2, dof, b);
  }

  graph_.assignEdges(start_idx, std::move(edges));
  if (!b) ROS_WARN("No edges between user input points at index %lu and %lu", start_idx, end_idx);
}

template<typename EdgeBuilder>
std::vector<LadderGraph::EdgeList> PlanningGraph::calculateEdgeWeights(EdgeBuilder&& builder, const std::vector<double>& start_joints,
                                                                       const std::vector<double>& end_joints, const size_t dof,
                                                                       bool& has_edges) const
{
  const auto from_size = start_joints.size();
  const auto to_size = end_joints.size();
  const auto n_start_points = from_size / dof;
  const auto n_end_points = to_size / dof;

  for (size_t i = 0; i < n_start_points; i++) // from rung
  {
    const auto start_index = i * dof;

    for (size_t j = 0; j < n_end_points; j++) // to rung
    {
      const auto end_index = j * dof;

      builder.consider(&start_joints[start_index], &end_joints[end_index], j);
    }

    builder.next(i);
  }

  has_edges = builder.hasEdges();
  return std::move(builder.result());
}

} /* namespace descartes_planner */
