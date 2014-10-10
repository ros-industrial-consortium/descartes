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
#include "descartes_trajectory_planning/trajectory_pt.h"
#include "descartes_trajectory_planning/cart_trajectory_pt.h"

#include <map>
#include <vector>

namespace descartes
{

struct JointVertex
{
	int index;
};
struct JointEdge
{
	int joint_start;
	int joint_end;
	double transition_cost;
};

struct CartesianPointRelationship
{
	int id;
	int id_previous;
	int id_next;
};

typedef boost::adjacency_list< boost::listS,            /*edge container*/
                               boost::vecS,             /*vertex_container*/
                               boost::directedS,        /*graph type*/
                               JointVertex,                  /*vertex structure*/
                               JointEdge                    /*edge structure*/
                             > DirectedGraph;

typedef boost::graph_traits<DirectedGraph>::vertex_iterator VertexIterator;
typedef boost::graph_traits<DirectedGraph>::edge_iterator EdgeIterator;
typedef boost::graph_traits<DirectedGraph>::out_edge_iterator OutEdgeIterator;




class PlanningGraph
{
public:
	// TODO: add constructor that takes RobotState as param
  PlanningGraph() {};
  virtual ~PlanningGraph() {};

  /** @brief initial population of graph trajectory elements
   * @param points list of trajectory points to be used to construct the graph
   * @param cartesianPointLinks for each TrajectoryPt in the points list, a struct that indicates order of the points
   */
  bool insertGraph(std::vector<CartTrajectoryPt> *points, std::vector<CartesianPointRelationship> *cartesianPointLinks);


  // TODO: addTrajectory
  // TODO: modifyTrajectory
  // TODO: removeTrajectory

  /** @brief Calculate and return the shortest path from the given joint solution indices
   * @param startIndex The index of the joint solution at which to start
   * @param endIndex The index of the joint solution at which to end
   * @param cost The cost of the returned path
   * @param path The sequence of points (joint solutions) for the path (TODO: change to JointTrajectoryPt?)
   * @return True if a valid path is found
   */
  bool getShortestPathJointToJoint(int startIndex, int endIndex, double &cost, std::list<int> &path);
  // TODO: 'overloaded' requests depending on source and destination
  //bool GetShortestPathJointToCartesian(int startIndex, int endIndex, double &cost, std::vector<TrajectoryPt> &path);
  //bool GetShortestPathCartesianToCartesian(int startIndex, int endIndex, double &cost, std::vector<TrajectoryPt> &path);

  /**@brief Utility function for printing the graph to the console
   * NOTE: should add other formats for output
   */
  void printGraph();

private:
  moveit::core::RobotState *robot_state_;

  DirectedGraph dg_;

  /** @brief DEBUG function for getting edge weights */
  double randomDouble(double min, double max);

  // NOTE: both Cartesian Points and Joint Points/solutions extend a base TrajectoryPt type
  //       and include an accessor to both formats

  // maintains an order to the Cartesian points list
  std::map<int, CartesianPointRelationship> cartesian_point_link_;

  // map from ID to Cartesian Coordinate point (these can be joint solutions also?)
  // NOTE: if this can be JointTrajectoryPt, make this a map to pointers (cannot create a map to the abstract TrajectoryPt object type)
  std::map<int, CartTrajectoryPt> trajectory_point_map_;

  // each JointSolution is a vertex in the graph, one or more of these will exist for each element in trajectory_point_map
  std::map<int, std::vector<double> > joint_solutions_map_;

  // map from Cartesian Point ID to applicable joint solutions per point
  std::map<int, std::list<int> > trajectory_point_to_joint_solutions_map_;

  /** @brief (Re)create the list of joint solutions from the given TrajectoryPt list */
  bool calculateJointSolutions();

  /** @brief (Re)populate the edge list for the graph from the list of joint solutions */
  bool calculateEdgeWeights(std::list<JointEdge> *edges);

  /** @brief (Re)create the actual graph structure from the list of joint solutions (vertices) and transition costs (edges) */
  bool populateGraph(std::list<JointEdge> *edges);
};

} /* namespace descartes */
#endif /* PLANNING_GRAPH_H_ */
