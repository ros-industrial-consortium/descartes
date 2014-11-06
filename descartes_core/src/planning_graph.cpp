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
 */

#include "descartes_core/planning_graph.h"

#include <stdio.h>
#include <iomanip>
#include <iostream>
#include <utility>
#include <algorithm>
#include <fstream>

#include <boost/uuid/uuid_io.hpp> // streaming operators
#include <boost/graph/dijkstra_shortest_paths.hpp>

namespace descartes_core
{

PlanningGraph::PlanningGraph(RobotModelConstPtr &model)
{
  robot_model_ = model;
}

PlanningGraph::~PlanningGraph()
{
  if(cartesian_point_link_)
  {
    delete cartesian_point_link_;
  }
}

bool PlanningGraph::insertGraph(std::vector<TrajectoryPtPtr> *points)
{
  // validate input
  if (!points)
  {
    // one or both are null
    logError("points == null. Cannot initialize graph with null list.");
    return false;
  }
  if (points->size() == 0)
  {
    // one or both have 0 elements
    logError("points.size == 0. Cannot initialize graph with 0 elements.");
    return false;
  }

  cartesian_point_link_ = new std::map<TrajectoryPt::ID, CartesianPointInformation>();

  TrajectoryPt::ID previous_id;

  // input is valid, copy to local maps that will be maintained by the planning graph
  for (std::vector<TrajectoryPtPtr>::iterator point_iter = points->begin(); point_iter != points->end(); point_iter++)
  {
    (*cartesian_point_link_)[point_iter->get()->getID()].source_trajectory_ = (*point_iter);
    CartesianPointRelationship *point_link = new CartesianPointRelationship();
    point_link->id = point_iter->get()->getID();

    // if the previous_id exists, set it's next_id to the new id
    if (cartesian_point_link_->find(previous_id) != cartesian_point_link_->end())
    {
      (*cartesian_point_link_)[previous_id].links_.id_next = point_link->id;
      point_link->id_previous = previous_id;
    }

    // set the new current point link
    point_link->id_previous = previous_id;

    // the new one becomes the previous_id
    previous_id = point_link->id;

    // save the point_link structure to the map
    (*cartesian_point_link_)[point_link->id].links_ = *point_link;
  }

  // after populating maps above (presumably from cartesian trajectory points), calculate (or query) for all joint trajectories
  if (!calculateJointSolutions())
  {
    // failed to get joint trajectories
    logError("unable to calculate joint trajectories for input points");
    return false;
  }

  if (!populateGraphVertices())
  {
    logError("unable to populate graph from input points");
    return false;
  }

  // after obtaining joint trajectories, calculate each edge weight between adjacent joint trajectories
  // edge list can be local here, it needs to be passed to populate the graph but not maintained afterwards
  std::list<JointEdge> edges;
  if (!calculateEdgeWeights(edges))
  {
    // failed to get edge weights
    logError("unable to calculate edge weight of joint transitions for joint trajectories");
    return false;
  }

  // from list of joint trajectories (vertices) and edges (edges), construct the actual graph
  if (!populateGraphEdges(edges))
  {
    // failed to create graph
    logError("unable to populate graph from calculated edges");
    return false;
  }

  // SUCCESS! now go do something interesting with the graph
  return true;
}

bool PlanningGraph::addTrajectory(TrajectoryPtPtr point, TrajectoryPt::ID previous_id, TrajectoryPt::ID next_id)
{
  if (previous_id.is_nil() && next_id.is_nil())
  {
    // unable to add a point with no forward or backward connections
    logError("unable to add a point that is not connected to one or more existing points");
    return false;
  }

  if (!previous_id.is_nil() && cartesian_point_link_->find(previous_id) == cartesian_point_link_->end())
  {
    // unable to find referenced previous_id
    logError("unable to find previous point");
    return false;
  }
  if (!next_id.is_nil() && cartesian_point_link_->find(next_id) == cartesian_point_link_->end())
  {
    // unable to find referenced next_id
    logError("unable to find next point");
  }

  CartesianPointRelationship *point_link = new CartesianPointRelationship();
  point_link->id = point->getID();
  point_link->id_next = next_id;
  point_link->id_previous = previous_id;

  // remove graph edges from each joint at [id_previous] to joint at [id_next]
  std::list<TrajectoryPt::ID> start_joint_ids = (*cartesian_point_link_)[previous_id].joints_;
  std::list<TrajectoryPt::ID> end_joint_ids = (*cartesian_point_link_)[next_id].joints_;
  for (std::list<TrajectoryPt::ID>::iterator start_joint_iter = start_joint_ids.begin();
      start_joint_iter != start_joint_ids.end(); start_joint_iter++)
  {
    for (std::list<TrajectoryPt::ID>::iterator end_joint_iter = end_joint_ids.begin();
        end_joint_iter != end_joint_ids.end(); end_joint_iter++)
    {
      boost::remove_edge(joint_solutions_map_[*start_joint_iter].second, joint_solutions_map_[*end_joint_iter].second,
                         dg_);
    }
  }

  // TODO: get joint poses from new trajectory point (unique id?) and
  //      add to trajectory_point_to_joint_solutions_map_ and to joint_solutions_map_

  // TODO: calculate weights from id_previous to id
  // TODO: calculate weights from id to id_next

//   for (std::list<int>::iterator start_joint_iter = start_joint_ids.begin(); start_joint_iter != start_joint_ids.end();
//   start_joint_iter++)
//   {
//   for (std::list<int>::iterator end_joint_iter = end_joint_ids.begin(); end_joint_iter != end_joint_ids.end();
//   end_joint_iter++)
//   {
//   double transition_cost;
//   // TODO: Make a call to somewhere that takes to JointTrajectoryPts (std::vector<double>) to get a single weight value back
//   std::vector<double> start_joint = joint_solutions_map_[*start_joint_iter];
//   std::vector<double> end_joint = joint_solutions_map_[*end_joint_iter];
//   transition_cost = randomDouble(.5, 5.0);
//   JointEdge *edge = new JointEdge();
//   edge->joint_start = *start_joint_iter;
//   edge->joint_end = *end_joint_iter;
//   edge->transition_cost = transition_cost;
//
//   edges->push_back(*edge);
//   }
//   }

  // TODO: add new vertices to graph

//   for (std::map<int, std::vector<double> >::iterator joint_iter = joint_solutions_map_.begin();
//   joint_iter != joint_solutions_map_.end(); joint_iter++)
//   {
//   DirectedGraph::vertex_descriptor v = boost::add_vertex(dg_);
//   dg_[v].id = joint_iter->first;
//   }

  // TODO: add new weights/edges to graph

//   for (std::list<JointEdge>::iterator edge_iter = edges->begin(); edge_iter != edges->end(); edge_iter++)
//   {
//   DirectedGraph::edge_descriptor e;
//   bool b;
//   // add graph links for structure
//   boost::tie(e, b) = boost::add_edge(edge_iter->joint_start, edge_iter->joint_end, dg_);
//   // populate edge fields
//   dg_[e].transition_cost = edge_iter->transition_cost;
//   dg_[e].joint_start = edge_iter->joint_start;
//   dg_[e].joint_end = edge_iter->joint_end;
//   }

  // if not adding at the beginning, update the previous to point to the new point
  if (cartesian_point_link_->find(previous_id) != cartesian_point_link_->end())
  {
    (*cartesian_point_link_)[previous_id].links_.id_next = point_link->id;
  }
  // if not updating the end, update the next to point to the new point
  if (cartesian_point_link_->find(next_id) != cartesian_point_link_->end())
  {
    (*cartesian_point_link_)[next_id].links_.id_previous = point_link->id;
  }
  // save the new point_link structure to the map
  (*cartesian_point_link_)[point_link->id].links_ = *point_link;

  // save the actual trajectory point into the map
  (*cartesian_point_link_)[point_link->id].source_trajectory_ = point;
  //trajectory_point_map_[point->getID()] = point;
}

bool PlanningGraph::modifyTrajectory(TrajectoryPtPtr point)
{
  TrajectoryPt::ID modify_id = point.get()->getID();
  if (modify_id.is_nil())
  {
    // unable to modify a point with nil ID
    logError("unable to modify a point with nil ID");
    return false;
  }
  if (cartesian_point_link_->find(modify_id) == cartesian_point_link_->end())
  {
    // unable to find cartesian point with ID:
    logError("unable to find cartesian point link with ID: %s", modify_id);
    return false;
  }
  // TODO: complete modify function
}

bool PlanningGraph::findStartVertices(std::list<int> &start_points)
{
  std::pair<VertexIterator, VertexIterator> vi = boost::vertices(dg_);
  for (VertexIterator vert_iter = vi.first; vert_iter != vi.second; ++vert_iter)
  {
    DirectedGraph::vertex_descriptor jv = *vert_iter;

    std::pair<InEdgeIterator, InEdgeIterator> in_ei = boost::in_edges(jv, dg_);
    if (in_ei.first == in_ei.second)
    {
      // debug
      logDebug("Graph start node: %d", jv);
      start_points.push_back(jv);
    }
  }
  return !start_points.empty();
}

bool PlanningGraph::findEndVertices(std::list<int> &end_points)
{
  std::pair<VertexIterator, VertexIterator> vi = boost::vertices(dg_);
  for (VertexIterator vert_iter = vi.first; vert_iter != vi.second; ++vert_iter)
  {
    DirectedGraph::vertex_descriptor jv = *vert_iter;

    std::pair<OutEdgeIterator, OutEdgeIterator> ei = boost::out_edges(jv, dg_);
    if (ei.first == ei.second)
    {
      logDebug("Graph end node: %d", jv);
      end_points.push_back(jv);
    }
  }
  return !end_points.empty();
}

bool PlanningGraph::getShortestPath(double &cost, std::list<JointTrajectoryPt> &path)
{
  std::list<int> start_points;
  findStartVertices(start_points);
  std::list<int> end_points;
  findEndVertices(end_points);

  size_t num_vert = boost::num_vertices(dg_);

  std::vector<TrajectoryPt::ID> vertex_index_map(num_vert);
  std::pair<VertexIterator, VertexIterator> vi = boost::vertices(dg_);
  int i = 0;
  for (VertexIterator vert_iter = vi.first; vert_iter != vi.second; ++vert_iter)
  {
    DirectedGraph::vertex_descriptor jv = *vert_iter;
    vertex_index_map[i++] = dg_[jv].id;
  }

  double lowest_weight = std::numeric_limits<double>::max();

  for (std::list<int>::iterator start = start_points.begin(); start != start_points.end(); start++)
  {
    for (std::list<int>::iterator end = end_points.begin(); end != end_points.end(); end++)
    {
      // initialize vectors to be used by dijkstra
      std::vector<int> predecessors(num_vert);
      std::vector<double> weights(num_vert, std::numeric_limits<double>::max());

      dijkstra_shortest_paths(
          dg_,
          *start,
          weight_map(get(&JointEdge::transition_cost, dg_)).distance_map(
              boost::make_iterator_property_map(weights.begin(), get(boost::vertex_index, dg_))).predecessor_map(
              &predecessors[0]));

      // actual weight(cost) from start_id to end_id
      double weight = weights[*end];

      // if the weight of this path of less than a previous one, replace the return path
      if (weight < lowest_weight)
      {
        cost = lowest_weight;
        path.clear();
        // Add the destination point.
        int current = *end;
        path.push_front(joint_solutions_map_[vertex_index_map[current]].first);

        // Starting from the destination point step through the predecessor map
        // until the source point is reached.
        while (current != *start)
        {
          current = predecessors[current];
          path.push_front(joint_solutions_map_[vertex_index_map[current]].first);
        }
      }
    }
  }

  if (lowest_weight < std::numeric_limits<double>::max())
  {
    return true;
  }
  else
  {
    logError("unable to find a valid path");
    return false;
  }
}

// TODO: optionally output this to a .DOT file (viewable in GraphVIZ or comparable)
void PlanningGraph::printGraph()
{
  std::stringstream ss;
  ss << "GRAPH VERTICES (" << num_vertices(dg_) << "): \n";
  std::pair<VertexIterator, VertexIterator> vi = vertices(dg_);
  for (VertexIterator vert_iter = vi.first; vert_iter != vi.second; ++vert_iter)
  {
    DirectedGraph::vertex_descriptor jv = *vert_iter;

    ss << "Vertex: " << dg_[jv].id;
    std::pair<OutEdgeIterator, OutEdgeIterator> out_ei = out_edges(jv, dg_);
    ss << " -> {";
    for (OutEdgeIterator out_edge = out_ei.first; out_edge != out_ei.second; ++out_edge)
    {
      DirectedGraph::edge_descriptor e = *out_edge;
      ss << dg_[e].joint_end << ", ";
    }
    ss << "}\n";
  }
  logDebug("%s", ss.str().c_str());

  ss.str("");
  for (VertexIterator vert_iter = vi.first; vert_iter != vi.second; ++vert_iter)
  {
    DirectedGraph::vertex_descriptor jv = *vert_iter;

    std::pair<InEdgeIterator, InEdgeIterator> in_ei = in_edges(jv, dg_);

    ss << "{";
    for (InEdgeIterator in_edge = in_ei.first; in_edge != in_ei.second; ++in_edge)
    {
      DirectedGraph::edge_descriptor e = *in_edge;
      ss << source(e, dg_) << ", ";
    }
    ss << "} -> ";
    ss << "Vertex (" << jv << "): " << dg_[jv].id << "\n";
  }
  logDebug("%s", ss.str().c_str());

  ss.str("");
  ss << "GRAPH EDGES (" << num_edges(dg_) << "): \n";
  //Tried to make this section more clear, instead of using tie, keeping all
  //the original types so it's more clear what is going on
  std::pair<EdgeIterator, EdgeIterator> ei = edges(dg_);
  for (EdgeIterator edge_iter = ei.first; edge_iter != ei.second; ++edge_iter)
  {
    DirectedGraph::vertex_descriptor jv = source(*edge_iter, dg_);
    DirectedGraph::edge_descriptor e = *out_edges(jv, dg_).first;

    DirectedGraph::edge_descriptor e2 = *edge_iter;

    ss << "(" << source(*edge_iter, dg_) << ", " << target(*edge_iter, dg_) << "): cost: " << dg_[e2].transition_cost
        << "\n";
  }

  ss << "\n";
  logDebug("%s", ss.str().c_str());

  ss.str("");
}

bool PlanningGraph::calculateJointSolutions()
{
  if (joint_solutions_map_.size() > 0)
  {
    // existing joint solutions... clear the list?
    logWarn("existing joint solutions found, clearing map");
    joint_solutions_map_.clear();
  }

  // for each TrajectoryPt, get the available joint solutions
  for (std::map<TrajectoryPt::ID, CartesianPointInformation>::iterator trajectory_iter = cartesian_point_link_->begin();
      trajectory_iter != cartesian_point_link_->end(); trajectory_iter++)
  {
    // TODO: copy this block to a function that can be used by add and modify
    /*************************/
    std::list<TrajectoryPt::ID> *traj_solutions = new std::list<TrajectoryPt::ID>();
    std::vector<std::vector<double> > joint_poses;
    trajectory_iter->second.source_trajectory_.get()->getJointPoses(*robot_model_, joint_poses);

    if (joint_poses.size() == 0)
    {
      logWarn("no joint solution for this point... potential discontinuity in the graph");
    }
    else
    {
      for (std::vector<std::vector<double> >::iterator joint_pose_iter = joint_poses.begin();
          joint_pose_iter != joint_poses.end(); joint_pose_iter++)
      {
        //get UUID from JointTrajPt (convert from std::vector<double>)
        JointTrajectoryPt *new_pt = new JointTrajectoryPt(*joint_pose_iter);
        traj_solutions->push_back(new_pt->getID());
        JointGraphVertexPair *joint_vertex = new JointGraphVertexPair();
        joint_vertex->first = *new_pt;
        joint_solutions_map_[new_pt->getID()] = *joint_vertex;
      }
    }
    trajectory_iter->second.joints_ == *traj_solutions;
    /*************************/
  }

  return true;
}

bool PlanningGraph::calculateEdgeWeights(std::list<JointEdge> &edges)
{
  if (cartesian_point_link_->size() == 0)
  {
    // no linkings of cartesian points
    logError("no trajectory point links defined");
    return false;
  }
  if (joint_solutions_map_.size() == 0)
  {
    // no joint solutions to calculate transitions for
    logError("no joint solutions available");
    return false;
  }

  for (std::map<TrajectoryPt::ID, CartesianPointInformation>::iterator cart_link_iter = cartesian_point_link_->begin();
      cart_link_iter != cartesian_point_link_->end(); cart_link_iter++)
  {
    TrajectoryPt::ID start_cart_id = cart_link_iter->first; // should be equal to cart_link_iter->second.links_.id
    TrajectoryPt::ID end_cart_id = cart_link_iter->second.links_.id_next;

    std::list<TrajectoryPt::ID> start_joint_ids = cart_link_iter->second.joints_;
    std::list<TrajectoryPt::ID> end_joint_ids = (*cartesian_point_link_)[end_cart_id].joints_;

    for (std::list<TrajectoryPt::ID>::iterator start_joint_iter = start_joint_ids.begin();
        start_joint_iter != start_joint_ids.end(); start_joint_iter++)
    {
      for (std::list<TrajectoryPt::ID>::iterator end_joint_iter = end_joint_ids.begin();
          end_joint_iter != end_joint_ids.end(); end_joint_iter++)
      {
        double transition_cost;
        // TODO: Make a call to somewhere that takes to JointTrajectoryPts (std::vector<double>) to get a single weight value back
        JointTrajectoryPt start_joint = joint_solutions_map_[*start_joint_iter].first;
        JointTrajectoryPt end_joint = joint_solutions_map_[*end_joint_iter].first;

        transition_cost = linearWeight(start_joint, end_joint);
        JointEdge *edge = new JointEdge();
        edge->joint_start = *start_joint_iter;
        edge->joint_end = *end_joint_iter;
        edge->transition_cost = transition_cost;

        edges.push_back(*edge);
      }
    }
  }

  return !edges.empty();
}

bool PlanningGraph::populateGraphVertices()
{
  if (joint_solutions_map_.size() == 0)
  {
    // no joints (vertices)
    logError("no joint solutions defined, thus no graph vertices");
    return false;
  }

  for (std::map<TrajectoryPt::ID, JointGraphVertexPair>::iterator joint_iter = joint_solutions_map_.begin();
      joint_iter != joint_solutions_map_.end(); joint_iter++)
  {
    DirectedGraph::vertex_descriptor v = boost::add_vertex(dg_);
    dg_[v].id = joint_iter->first;
    joint_iter->second.second = v;
  }

  return true;
}

bool PlanningGraph::populateGraphEdges(const std::list<JointEdge> &edges)
{
  if (edges.size() == 0)
  {
    // no edges
    logError("no graph edges defined");
    return false;
  }

  for (std::list<JointEdge>::const_iterator edge_iter = edges.begin(); edge_iter != edges.end(); edge_iter++)
  {
    DirectedGraph::edge_descriptor e;
    bool b;
    // add graph links for structure
    boost::tie(e, b) = boost::add_edge(joint_solutions_map_[edge_iter->joint_start].second,
                                       joint_solutions_map_[edge_iter->joint_end].second, dg_);
    // populate edge fields
    dg_[e].transition_cost = edge_iter->transition_cost;
    dg_[e].joint_start = edge_iter->joint_start;
    dg_[e].joint_end = edge_iter->joint_end;
  }

  return true;
}

double PlanningGraph::linearWeight(JointTrajectoryPt start, JointTrajectoryPt end)
{
  std::vector<std::vector<double> > joint_poses_start;
  start.getJointPoses(*robot_model_, joint_poses_start);

  std::vector<std::vector<double> > joint_poses_end;
  end.getJointPoses(*robot_model_, joint_poses_end);

  // each should only return one
  if (joint_poses_start.size() == 1 && joint_poses_end.size() == 1)
  {
    std::vector<double> start_vector = joint_poses_start[0];
    std::vector<double> end_vector = joint_poses_end[0];
    if (start_vector.size() == end_vector.size())
    {
      double vector_diff = 0;
      for (int i = 0; i < start_vector.size(); i++)
      {
        vector_diff += abs(end_vector[i] - start_vector[i]);
      }
      return vector_diff;
    }
    else
    {
      logWarn("unequal joint pose vector lengths");
      return std::numeric_limits<double>::max();
    }
  }
  else
  {
    logWarn("invalid joint pose(s) found");
    return std::numeric_limits<double>::max();
  }
}
} /* namespace descartes_core */
