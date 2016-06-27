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
 */

#ifndef PLANNING_GRAPH_H_
#define PLANNING_GRAPH_H_

#include <boost/graph/adjacency_list.hpp>
#include <boost/function.hpp>
#include "descartes_core/trajectory_pt.h"
#include "descartes_trajectory/cart_trajectory_pt.h"
#include "descartes_trajectory/joint_trajectory_pt.h"

#include <map>
#include <vector>

namespace descartes_planner
{
struct JointVertex
{
  descartes_core::TrajectoryPt::ID id;
};
struct JointEdge
{
  descartes_core::TrajectoryPt::ID joint_start;
  descartes_core::TrajectoryPt::ID joint_end;
  double transition_cost;
};

struct CartesianPointRelationship
{
  descartes_core::TrajectoryPt::ID id;
  descartes_core::TrajectoryPt::ID id_previous;
  descartes_core::TrajectoryPt::ID id_next;
};


// A "joint-trajectory-point"-lite class. Behaves as if it was a joint point but does
// not contain the structures for things not used by the planning such as discretization
// and coordinate frames
class InternalJointPt
{
public:
  InternalJointPt() {} // map requires a default constructor
  InternalJointPt(const std::vector<double>& joints,
                  const descartes_core::TimingConstraint& timing = descartes_core::TimingConstraint())
    : joints_(joints)
    , timing_(timing)
    , id_ (descartes_core::TrajectoryID::make_id())
  {}

  const std::vector<double>& nominal() const { return joints_; }
  const descartes_core::TimingConstraint& getTiming() const { return timing_; }
  const descartes_core::TrajectoryID getID() const { return id_; }

private:
  std::vector<double> joints_;
  descartes_core::TimingConstraint timing_;
  descartes_core::TrajectoryID id_;
};


typedef boost::adjacency_list<boost::listS,          /*edge container*/
                              boost::vecS,           /*vertex_container*/
                              boost::bidirectionalS, /*allows in_edge and out_edge*/
                              JointVertex,           /*vertex structure*/
                              JointEdge              /*edge structure*/
                              > JointGraph;

typedef boost::graph_traits<JointGraph>::vertex_iterator VertexIterator;
typedef boost::graph_traits<JointGraph>::edge_iterator EdgeIterator;
typedef boost::graph_traits<JointGraph>::out_edge_iterator OutEdgeIterator;
typedef boost::graph_traits<JointGraph>::in_edge_iterator InEdgeIterator;

typedef boost::function<double(const std::vector<double> &, const std::vector<double> &)> CostFunction;

struct CartesianPointInformation
{
  CartesianPointRelationship links_;
  descartes_core::TrajectoryPtPtr source_trajectory_;
  std::vector<descartes_core::TrajectoryPt::ID> joints_;
};

typedef std::map<descartes_core::TrajectoryPt::ID, CartesianPointInformation> CartesianMap;
typedef std::map<descartes_core::TrajectoryPt::ID, InternalJointPt> JointMap;
typedef std::map<descartes_core::TrajectoryPt::ID, JointGraph::vertex_descriptor> VertexMap;

class PlanningGraph
{
public:
  // TODO: add constructor that takes RobotState as param
  PlanningGraph(descartes_core::RobotModelConstPtr model);

  PlanningGraph(descartes_core::RobotModelConstPtr model, CostFunction cost_function_callback);

  virtual ~PlanningGraph();

  /** \brief Clear all previous graph data */
  void clear();

  /** @brief initial population of graph trajectory elements
   * @param points list of trajectory points to be used to construct the graph
   * @return True if the graph was successfully created
   */
  bool insertGraph(const std::vector<descartes_core::TrajectoryPtPtr> *points);

  /** @brief adds a single trajectory point to the graph
   * @param point The new point to add to the graph
   * @return True if the point was successfully added
   */
  bool addTrajectory(descartes_core::TrajectoryPtPtr point, descartes_core::TrajectoryPt::ID previous_id,
                     descartes_core::TrajectoryPt::ID next_id);

  bool modifyTrajectory(descartes_core::TrajectoryPtPtr point);

  bool removeTrajectory(descartes_core::TrajectoryPtPtr point);

  bool getCartesianTrajectory(std::vector<descartes_core::TrajectoryPtPtr> &traj);

  /** @brief Calculate and return the shortest path from the given joint solution indices
   * @param startIndex The index of the joint solution at which to start
   * @param endIndex The index of the joint solution at which to end
   * @param cost The cost of the returned path
   * @param path The sequence of points (joint solutions) for the path (TODO: change to JointTrajectoryPt?)
   * @return True if a valid path is found
   */
  bool getShortestPath(double &cost, std::list<descartes_trajectory::JointTrajectoryPt> &path);
  // TODO: 'overloaded' requests depending on source and destination
  // bool GetShortestPathJointToCartesian(int startIndex, int endIndex, double &cost,
  // std::vector<descartes_core::TrajectoryPt> &path);
  // bool GetShortestPathCartesianToCartesian(int startIndex, int endIndex, double &cost,
  // std::vector<descartes_core::TrajectoryPt> &path);

  /**@brief Utility function for printing the graph to the console
   * NOTE: should add other formats for output
   */
  void printGraph();
  void printMaps();

  descartes_core::RobotModelConstPtr getRobotModel();

  CartesianMap getCartesianMap() const;

  const JointMap &getJointMap() const;

  const JointGraph &getGraph() const;

  /** @brief simple function to iterate over all graph vertices to find ones that do not have an incoming edge */
  bool findStartVertices(std::vector<JointGraph::vertex_descriptor> &start_points) const;

  /** @brief simple function to iterate over all graph vertices to find ones that do not have an outgoing edge */
  bool findEndVertices(std::vector<JointGraph::vertex_descriptor> &end_points) const;

protected:
  descartes_core::RobotModelConstPtr robot_model_;

  CostFunction custom_cost_function_;

  JointGraph dg_;

  int recalculateJointSolutionsVertexMap(
      std::map<descartes_core::TrajectoryPt::ID, JointGraph::vertex_descriptor> &joint_vertex_map) const;

  /**
   * @brief A pair indicating the validity of the edge, and if valid, the cost associated
   *        with that edge
   */
  typedef std::pair<bool, double> EdgeWeightResult;

  /** @brief function for computing edge weight based on specified cost function */
  EdgeWeightResult edgeWeight(const InternalJointPt &start,
                              const InternalJointPt &end) const;

  // NOTE: both Cartesian Points and Joint Points/solutions extend a base descartes_core::TrajectoryPt type
  //       and include an accessor to both formats

  // maintains the original (Cartesian) points list along with link information and associated joint trajectories per
  // point
  CartesianMap *cartesian_point_link_;

  // maintains a map of joint solutions with it's corresponding graph vertex_descriptor
  //   one or more of these will exist for each element in trajectory_point_map
  JointMap joint_solutions_map_;

  /** @brief (Re)create the list of joint solutions from the given descartes_core::TrajectoryPt list */
  bool calculateJointSolutions(const std::vector<descartes_core::TrajectoryPtPtr> &points,
                               std::vector<std::vector<InternalJointPt>> &poses);

  /** @brief (Re)create the actual graph nodes(vertices) from the list of joint solutions (vertices) */
  bool populateGraphVertices(const std::vector<descartes_core::TrajectoryPtPtr> &points,
                             std::vector<std::vector<InternalJointPt>> &poses);

  /** @brief calculate weights fro each start point to each end point */
  bool calculateEdgeWeights(const std::vector<descartes_core::TrajectoryPt::ID> &start_joints,
                            const std::vector<descartes_core::TrajectoryPt::ID> &end_joints,
                            std::vector<JointEdge> &edge_results);

  bool calculateEdgeWeights(const std::vector<InternalJointPt> &start_joints,
                            const std::vector<InternalJointPt> &end_joints,
                            std::vector<JointEdge> &edge_results) const;

  /** @brief (Re)populate the edge list for the graph from the list of joint solutions */
  bool calculateAllEdgeWeights(const std::vector<std::vector<InternalJointPt>> &poses,
                               std::vector<JointEdge> &edges);

  /** @brief (Re)create the actual graph structure from the list of transition costs (edges) */
  bool populateGraphEdges(const std::vector<JointEdge> &edges);
};

} /* namespace descartes_planner */

#endif /* PLANNING_GRAPH_H_ */
