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

  bool removeTrajectory(const descartes_core::TrajectoryPt::ID& point);

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

} /* namespace descartes_planner */

#endif /* PLANNING_GRAPH_H_ */
