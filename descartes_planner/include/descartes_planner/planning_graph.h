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
 * Planning_graph.h
 *
 *  Created on: Jun 5, 2014
 *      Author: Dan Solomon
 *      Author: Jonathan Meyer
 */

#ifndef PLANNING_GRAPH_H_
#define PLANNING_GRAPH_H_

#include <boost/function.hpp>
#include "descartes_core/trajectory_pt.h"
#include "descartes_trajectory/cart_trajectory_pt.h"
#include "descartes_trajectory/joint_trajectory_pt.h"

#include "descartes_planner/ladder_graph.h"

namespace descartes_planner
{

typedef boost::function<double(const double*, const double*)> CostFunction;


class PlanningGraph
{
public:
  PlanningGraph(descartes_core::RobotModelConstPtr model, CostFunction cost_function_callback = CostFunction{});

  /** \brief Clear all previous graph data */
  void clear() { graph_.clear(); }

  /** @brief initial population of graph trajectory elements
   * @param points list of trajectory points to be used to construct the graph
   * @return True if the graph was successfully created
   */
  bool insertGraph(const std::vector<descartes_core::TrajectoryPtPtr>& points);

  /** @brief adds a single trajectory point to the graph
   * @param point The new point to add to the graph
   * @return True if the point was successfully added
   */
  bool addTrajectory(descartes_core::TrajectoryPtPtr point, descartes_core::TrajectoryPt::ID previous_id,
                     descartes_core::TrajectoryPt::ID next_id);

  bool modifyTrajectory(descartes_core::TrajectoryPtPtr point);

  bool removeTrajectory(descartes_core::TrajectoryPtPtr point);

  bool getShortestPath(double &cost, std::list<descartes_trajectory::JointTrajectoryPt> &path);

  const descartes_planner::LadderGraph& graph() const noexcept { return graph_; }

  descartes_core::RobotModelConstPtr getRobotModel() const { return robot_model_; }

protected:
  descartes_planner::LadderGraph graph_;
  descartes_core::RobotModelConstPtr robot_model_;
  CostFunction custom_cost_function_;

  /**
   * @brief A pair indicating the validity of the edge, and if valid, the cost associated
   *        with that edge
   */
  bool calculateJointSolutions(const descartes_core::TrajectoryPtPtr* points, const std::size_t count,
                               std::vector<std::vector<std::vector<double>>>& poses) const;

  /** @brief (Re)create the actual graph nodes(vertices) from the list of joint solutions (vertices) */
  bool populateGraphVertices(const std::vector<descartes_core::TrajectoryPtPtr> &points,
                             std::vector<std::vector<descartes_trajectory::JointTrajectoryPt>> &poses);

  void computeAndAssignEdges(const std::size_t start_idx, const std::size_t end_idx);


  template <typename EdgeBuilder>
  std::vector<LadderGraph::EdgeList> calculateEdgeWeights(EdgeBuilder&& builder,
                                                          const std::vector<double> &start_joints,
                                                          const std::vector<double> &end_joints,
                                                          const size_t dof,
                                                          bool& has_edges) const;

};

struct DefaultEdgesWithTime
{
 DefaultEdgesWithTime(const size_t n_start,
                      const size_t n_end,
                      const size_t dof,
                      const double upper_tm,
                      const std::vector<double>& joint_vel_limits)
    : results_(n_start)
    , edge_scratch_(n_end)
    , max_dtheta_(dof)
    , delta_buffer_(dof)
    , dof_(dof)
    , count_(0)
    , has_edges_(false)
  {
   std::transform(joint_vel_limits.cbegin(), joint_vel_limits.cend(), max_dtheta_.begin(), [upper_tm] (double v) {
                    return v * upper_tm;
                  });
  }

  inline void consider(const double* const start, const double* const stop, size_t index) noexcept
  {
    for (size_t i = 0; i < dof_; ++i)
    {
      delta_buffer_[i] = std::abs(start[i] - stop[i]);
      if (delta_buffer_[i] > max_dtheta_[i]) return;
    }

    auto cost = std::accumulate(delta_buffer_.cbegin(), delta_buffer_.cend(), 0.0);
    edge_scratch_[count_].cost = cost;
    edge_scratch_[count_].idx = static_cast<unsigned>(index);
    count_++;
  }

  inline void next(const size_t i)
  {
    results_[i].assign(edge_scratch_.cbegin(), edge_scratch_.cbegin() + count_);
    has_edges_ = has_edges_ || count_ > 0;
    count_ = 0;
  }

  inline std::vector<LadderGraph::EdgeList>& result() noexcept { return results_; }

  inline bool hasEdges() const noexcept { return has_edges_; }

  std::vector<LadderGraph::EdgeList> results_;
  LadderGraph::EdgeList edge_scratch_; // pre-allocated space to work in
  std::vector<double> max_dtheta_;
  std::vector<double> delta_buffer_;
  size_t dof_;
  unsigned count_;
  bool has_edges_;
};

struct CustomEdgesWithTime : public DefaultEdgesWithTime
{
  CustomEdgesWithTime(const size_t n_start,
                      const size_t n_end,
                      const size_t dof,
                      const double upper_tm,
                      const std::vector<double>& joint_vel_limits,
                      descartes_planner::CostFunction fn)
    : DefaultEdgesWithTime(n_start, n_end, dof, upper_tm, joint_vel_limits)
    , custom_cost_fn(fn)
  {}

  inline void consider(const double * const start, const double * const stop, size_t index) noexcept
  {
    for (size_t i = 0; i < dof_; ++i)
    {
      delta_buffer_[i] = std::abs(start[i] - stop[i]);
      if (delta_buffer_[i] > max_dtheta_[i]) return;
    }

    double cost = custom_cost_fn(start, stop);
    edge_scratch_[count_++] = {cost, static_cast<unsigned>(index)};
  }

  descartes_planner::CostFunction custom_cost_fn;
};

} /* namespace descartes_planner */

#endif /* PLANNING_GRAPH_H_ */
