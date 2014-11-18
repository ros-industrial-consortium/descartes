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

#include <ros/console.h>

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

  //printMaps();

  TrajectoryPt::ID previous_id = boost::uuids::nil_uuid();

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
      //point_link->id_previous = previous_id;
      logInform("PreviousID[%s].links_.id_next = %s",
                boost::uuids::to_string(previous_id).c_str(),
                boost::uuids::to_string(point_link->id).c_str());
    }
    else
    {
      logInform("PreviousID: %s was not found", boost::uuids::to_string(previous_id).c_str());
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
  if (!calculateAllEdgeWeights(edges))
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

  printGraph();
  printMaps();

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

  // save the new point_link structure to the map
  (*cartesian_point_link_)[point_link->id].links_ = *point_link;

  // if not adding at the beginning, update the previous to point to the new point
  //if (cartesian_point_link_->find(previous_id) != cartesian_point_link_->end())
  if(!previous_id.is_nil())
  {
    (*cartesian_point_link_)[previous_id].links_.id_next = point_link->id;
  }
  // if not updating the end, update the next to point to the new point
  //if (cartesian_point_link_->find(next_id) != cartesian_point_link_->end())
  if(!next_id.is_nil())
  {
    (*cartesian_point_link_)[next_id].links_.id_previous = point_link->id;
  }

  ROS_DEBUG("New ID: %s", boost::uuids::to_string(point_link->id).c_str());
  ROS_DEBUG("New Next ID: %s", boost::uuids::to_string(point_link->id_next).c_str());
  ROS_DEBUG("New Previous ID: %s", boost::uuids::to_string(point_link->id_previous).c_str());

  if(!previous_id.is_nil() && !next_id.is_nil())
  {
    // remove graph edges from each joint at [id_previous] to joint at [id_next]
    std::list<TrajectoryPt::ID> start_joint_ids = (*cartesian_point_link_)[previous_id].joints_;
    std::list<TrajectoryPt::ID> end_joint_ids = (*cartesian_point_link_)[next_id].joints_;
    for (std::list<TrajectoryPt::ID>::iterator start_joint_iter = start_joint_ids.begin();
        start_joint_iter != start_joint_ids.end(); start_joint_iter++)
    {
      for (std::list<TrajectoryPt::ID>::iterator end_joint_iter = end_joint_ids.begin();
          end_joint_iter != end_joint_ids.end(); end_joint_iter++)
      {
        std::cout << "Removing edge: " << *start_joint_iter << " -> " << *end_joint_iter << std::endl;
        boost::remove_edge(joint_solutions_map_[*start_joint_iter].second, joint_solutions_map_[*end_joint_iter].second,
                           dg_);
      }
    }
  }

  // get joint poses from new trajectory point (unique id)
  std::vector<std::vector<double> > joint_poses;
  point.get()->getJointPoses(*robot_model_, joint_poses);

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
      //traj_solutions->push_back(new_pt->getID());
      (*cartesian_point_link_)[point->getID()].joints_.push_back(new_pt->getID());
      JointGraphVertexPair *joint_vertex = new JointGraphVertexPair();
      joint_vertex->first = *new_pt;

      // insert new vertices into graph
      DirectedGraph::vertex_descriptor v = boost::add_vertex(dg_);
      dg_[v].id = new_pt->getID();
      joint_vertex->second = v;

      joint_solutions_map_[new_pt->getID()] = *joint_vertex;
    }
  }

  // save the list of joint solutions
  std::list<TrajectoryPt::ID> traj_solutions = (*cartesian_point_link_)[point->getID()].joints_;
  // save the actual trajectory point into the map
  (*cartesian_point_link_)[point->getID()].source_trajectory_ = point;

  ROS_INFO("SAVE CART POINT ID[%s]: %s",
           boost::uuids::to_string(point->getID()).c_str(),
           boost::uuids::to_string((*cartesian_point_link_)[point->getID()].source_trajectory_.get()->getID()).c_str());


  std::list<JointEdge> edges;
  // recalculate edges(previous -> this; this -> next)

  if(!previous_id.is_nil())
  {
    std::list<TrajectoryPt::ID> previous_joint_ids = (*cartesian_point_link_)[previous_id].joints_;
    calculateEdgeWeights(previous_joint_ids, traj_solutions, edges);
  }

  if(!next_id.is_nil())
  {
    std::list<TrajectoryPt::ID> next_joint_ids = (*cartesian_point_link_)[next_id].joints_;
    calculateEdgeWeights(traj_solutions, next_joint_ids, edges);
  }

  // insert new edges
  populateGraphEdges(edges);

  printGraph();
  printMaps();

  // simple test for now to see the new point is in the list
  return (cartesian_point_link_->find(point->getID()) != cartesian_point_link_->end());
}

bool PlanningGraph::modifyTrajectory(TrajectoryPtPtr point)
{
  TrajectoryPt::ID modify_id = point.get()->getID();
  ROS_INFO("Attempting to modify point:: %s", boost::uuids::to_string(modify_id).c_str());

  printMaps();

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
  // identify joint points at this cartesian point
  std::list<TrajectoryPt::ID> start_joint_ids = (*cartesian_point_link_)[modify_id].joints_;

  ROS_INFO("start_joint_ids.size = %d", (int)start_joint_ids.size());

  // remove edges
  for (std::list<TrajectoryPt::ID>::iterator start_joint_iter = start_joint_ids.begin();
      start_joint_iter != start_joint_ids.end(); start_joint_iter++)
  {
    // get the graph vertex descriptor
    DirectedGraph::vertex_descriptor jv = joint_solutions_map_[*start_joint_iter].second;

    ROS_INFO("jv: %d", (int)jv);

    // TODO: make this a standalone function to take a jv and return a list of edges to remove
    // remove out edges
    std::pair<OutEdgeIterator, OutEdgeIterator> out_ei = out_edges(jv, dg_);
    std::vector<DirectedGraph::edge_descriptor> to_remove;
    for (OutEdgeIterator out_edge = out_ei.first; out_edge != out_ei.second; ++out_edge)
    {
      DirectedGraph::edge_descriptor e = *out_edge;
      ROS_INFO("REMOVE OUTEDGE: %s -> %s",
               boost::uuids::to_string(dg_[e].joint_start).c_str(),
               boost::uuids::to_string(dg_[e].joint_end).c_str());
      to_remove.push_back(e);
    }

    // remove in edges
    std::pair<InEdgeIterator, InEdgeIterator> in_ei = in_edges(jv, dg_);
    for (InEdgeIterator in_edge = in_ei.first; in_edge != in_ei.second; ++in_edge)
    {
      DirectedGraph::edge_descriptor e = *in_edge;
      ROS_INFO("REMOVE INEDGE: %s -> %s",
               boost::uuids::to_string(dg_[e].joint_start).c_str(),
               boost::uuids::to_string(dg_[e].joint_end).c_str());
      to_remove.push_back(e);
    }

    for(std::vector<DirectedGraph::edge_descriptor>::iterator e_iter = to_remove.begin(); e_iter != to_remove.end(); e_iter++)
    {
      boost:remove_edge(*e_iter, dg_);
    }

    // remove the graph vertex and joint point
    boost::remove_vertex(jv, dg_);
    joint_solutions_map_.erase(*start_joint_iter);
  }

  // get new joint points for this cartesian
  std::list<TrajectoryPt::ID> *traj_solutions = new std::list<TrajectoryPt::ID>();
  std::vector<std::vector<double> > joint_poses;
  point.get()->getJointPoses(*robot_model_, joint_poses);

  if (joint_poses.size() == 0)
  {
    ROS_WARN("no joint solution for this point... potential discontinuity in the graph");
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

      // insert new vertices into graph
      DirectedGraph::vertex_descriptor v = boost::add_vertex(dg_);
      dg_[v].id = new_pt->getID();
      joint_vertex->second = v;

      joint_solutions_map_[new_pt->getID()] = *joint_vertex;
    }
  }
  (*cartesian_point_link_)[modify_id].joints_ == *traj_solutions;
  (*cartesian_point_link_)[modify_id].source_trajectory_ = point;
  // don't need to modify links


  std::list<JointEdge> edges;
  // recalculate edges(previous -> this; this -> next)
  TrajectoryPt::ID previous_cart_id = (*cartesian_point_link_)[modify_id].links_.id_previous; // should be equal to cart_link_iter->second.links_.id
  TrajectoryPt::ID next_cart_id = (*cartesian_point_link_)[modify_id].links_.id_next;

  std::list<TrajectoryPt::ID> previous_joint_ids = (*cartesian_point_link_)[previous_cart_id].joints_;
  std::list<TrajectoryPt::ID> next_joint_ids = (*cartesian_point_link_)[next_cart_id].joints_;

  calculateEdgeWeights(previous_joint_ids, *traj_solutions, edges);
  calculateEdgeWeights(*traj_solutions, next_joint_ids, edges);

  // insert new edges
  populateGraphEdges(edges);

  printGraph();
}

bool PlanningGraph::removeTrajectory(TrajectoryPtPtr point)
{
  TrajectoryPt::ID delete_id = point.get()->getID();
  ROS_INFO("Attempting to delete ID: %s", boost::uuids::to_string(delete_id).c_str());

  if (delete_id.is_nil())
  {
    // unable to modify a point with nil ID
    ROS_ERROR("unable to delete a point with nil ID");
    return false;
  }
  if (cartesian_point_link_->find(delete_id) == cartesian_point_link_->end())
  {
    // unable to find cartesian point with ID:
    ROS_ERROR("unable to find cartesian point link with ID: %s", boost::uuids::to_string(delete_id).c_str());
    return false;
  }
  // identify joint points at this cartesian point
  std::list<TrajectoryPt::ID> start_joint_ids = (*cartesian_point_link_)[delete_id].joints_;

  ROS_DEBUG("Attempting to delete edges from %d vertices", (int)start_joint_ids.size());

  // remove edges
  for (std::list<TrajectoryPt::ID>::iterator start_joint_iter = start_joint_ids.begin();
      start_joint_iter != start_joint_ids.end(); start_joint_iter++)
  {
    // get the graph vertex descriptor
    DirectedGraph::vertex_descriptor jv = joint_solutions_map_[*start_joint_iter].second;

    // remove out edges
    std::pair<OutEdgeIterator, OutEdgeIterator> out_ei = out_edges(jv, dg_);
    std::vector<DirectedGraph::edge_descriptor> to_remove;
    for (OutEdgeIterator out_edge = out_ei.first; out_edge != out_ei.second; ++out_edge)
    {
      DirectedGraph::edge_descriptor e = *out_edge;
      ROS_DEBUG("REMOVE OUTEDGE: %s -> %s",
               boost::uuids::to_string(dg_[e].joint_start).c_str(),
               boost::uuids::to_string(dg_[e].joint_end).c_str());
      to_remove.push_back(e);
    }

    // remove in edges
    std::pair<InEdgeIterator, InEdgeIterator> in_ei = in_edges(jv, dg_);
    for (InEdgeIterator in_edge = in_ei.first; in_edge != in_ei.second; ++in_edge)
    {
      DirectedGraph::edge_descriptor e = *in_edge;
      ROS_DEBUG("REMOVE INEDGE: %s -> %s",
               boost::uuids::to_string(dg_[e].joint_start).c_str(),
               boost::uuids::to_string(dg_[e].joint_end).c_str());
      to_remove.push_back(e);
    }

    for(std::vector<DirectedGraph::edge_descriptor>::iterator e_iter = to_remove.begin(); e_iter != to_remove.end(); e_iter++)
    {
      boost:remove_edge(*e_iter, dg_);
    }

    // remove the graph vertex and joint point
    boost::remove_vertex(jv, dg_);
    joint_solutions_map_.erase(*start_joint_iter);
  }

  // get previous_id and next_id from cartesian list
  CartesianPointRelationship links = (*cartesian_point_link_)[delete_id].links_;
  TrajectoryPt::ID previous_id = links.id_previous;
  TrajectoryPt::ID next_id = links.id_next;

  // change previous_cartesian link to next_id
  (*cartesian_point_link_)[previous_id].links_.id_next = next_id;
  // change next_cartesian link to previous_id
  (*cartesian_point_link_)[next_id].links_.id_previous = previous_id;

  // TODO: calculate edges from new previous to new next
  std::list<TrajectoryPt::ID> previous_joint_ids = (*cartesian_point_link_)[previous_id].joints_;
  std::list<TrajectoryPt::ID> next_joint_ids = (*cartesian_point_link_)[next_id].joints_;

  std::list<JointEdge> edges;

  calculateEdgeWeights(previous_joint_ids, next_joint_ids, edges);
  populateGraphEdges(edges);

  printGraph();

  return true;
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
      ROS_INFO("Graph start node: %d", (int)jv);
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
      ROS_INFO("Graph end node: %d", (int)jv);
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

  cost = std::numeric_limits<double>::max();

  for (std::list<int>::iterator start = start_points.begin(); start != start_points.end(); start++)
  {
    for (std::list<int>::iterator end = end_points.begin(); end != end_points.end(); end++)
    {
      //logInform("Checking path: S[%d] -> E[%d]", *start, *end);
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
      if (weight < cost)
      {
        cost = weight;
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

  if (cost < std::numeric_limits<double>::max())
  {
    return true;
  }
  else
  {
    logError("unable to find a valid path");
    return false;
  }
}

void PlanningGraph::printMaps()
{
  ROS_INFO("Number of points: %d", (int)cartesian_point_link_->size());

  for(std::map<TrajectoryPt::ID, CartesianPointInformation>::iterator c_iter = cartesian_point_link_->begin();
      c_iter != cartesian_point_link_->end(); c_iter++)
  {
    ROS_INFO("C_ID: %s [P_ID: %s -> N_ID: %s](Joints: %d)",
             boost::uuids::to_string(c_iter->first).c_str(),
             boost::uuids::to_string(c_iter->second.links_.id_previous).c_str(),
             boost::uuids::to_string(c_iter->second.links_.id_next).c_str(),
             (int)c_iter->second.joints_.size());
  }
}

// TODO: optionally output this to a .DOT file (viewable in GraphVIZ or comparable)
void PlanningGraph::printGraph()
{
  std::stringstream ss;
  ss << "GRAPH VERTICES (" << num_vertices(dg_) << "): ";
  ROS_INFO("%s", ss.str().c_str());
  std::pair<VertexIterator, VertexIterator> vi = vertices(dg_);
  ROS_INFO("Graph OutEdges:");
  for (VertexIterator vert_iter = vi.first; vert_iter != vi.second; ++vert_iter)
  {
    DirectedGraph::vertex_descriptor jv = *vert_iter;
    ss.str("");
    ss << "Vertex: (" << jv << ")";
    ss << dg_[jv].id;
    std::pair<OutEdgeIterator, OutEdgeIterator> out_ei = out_edges(jv, dg_);
    ss << " -> {";
    for (OutEdgeIterator out_edge = out_ei.first; out_edge != out_ei.second; ++out_edge)
    {
      DirectedGraph::edge_descriptor e = *out_edge;
      ss << dg_[e].joint_end << ", ";
    }
    ss << "}";
    ROS_INFO("%s", ss.str().c_str());
  }

  ROS_INFO("Graph InEdges:");
  for (VertexIterator vert_iter = vi.first; vert_iter != vi.second; ++vert_iter)
  {
    DirectedGraph::vertex_descriptor jv = *vert_iter;

    std::pair<InEdgeIterator, InEdgeIterator> in_ei = in_edges(jv, dg_);
    ss.str("");
    ss << "{";
    for (InEdgeIterator in_edge = in_ei.first; in_edge != in_ei.second; ++in_edge)
    {
      DirectedGraph::edge_descriptor e = *in_edge;
      ss << source(e, dg_) << ", ";
    }
    ss << "} -> ";
    ss << "Vertex (" << jv << "): " << dg_[jv].id << "\n";
    ROS_INFO("%s", ss.str().c_str());
  }

  ss.str("");
  ss << "GRAPH EDGES (" << num_edges(dg_) << "): \n";
  ROS_INFO("%s", ss.str().c_str());
  //Tried to make this section more clear, instead of using tie, keeping all
  //the original types so it's more clear what is going on
  std::pair<EdgeIterator, EdgeIterator> ei = edges(dg_);
  for (EdgeIterator edge_iter = ei.first; edge_iter != ei.second; ++edge_iter)
  {
    DirectedGraph::vertex_descriptor jv = source(*edge_iter, dg_);
    DirectedGraph::edge_descriptor e = *out_edges(jv, dg_).first;

    DirectedGraph::edge_descriptor e2 = *edge_iter;
    ss.str("");
    ss << "(" << source(*edge_iter, dg_) << ", " << target(*edge_iter, dg_) << "): cost: " << dg_[e2].transition_cost;
    ROS_INFO("%s", ss.str().c_str());
  }

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

    TrajectoryPt::ID tempID = trajectory_iter->first;
    ROS_INFO("CartID: %s: JointPoses count:%i", boost::uuids::to_string(tempID).c_str(), (int)(joint_poses.size()));

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
    trajectory_iter->second.joints_ = *traj_solutions;
    /*************************/
  }

  return true;
}

bool PlanningGraph::calculateAllEdgeWeights(std::list<JointEdge> &edges)
{
  if (cartesian_point_link_->size() == 0)
  {
    // no linkings of cartesian points
    logError("no trajectory point links defined");
    return false;
  }
  else
  {
    logInform("Found %i trajectory point links",cartesian_point_link_->size());
  }

  if (joint_solutions_map_.size() == 0)
  {
    // no joint solutions to calculate transitions for
    logError("no joint solutions available");
    return false;
  }
  else
  {
    logInform("Found %i joint solutions available",joint_solutions_map_.size());
  }

  for (std::map<TrajectoryPt::ID, CartesianPointInformation>::iterator cart_link_iter = cartesian_point_link_->begin();
      cart_link_iter != cartesian_point_link_->end(); cart_link_iter++)
  {
    TrajectoryPt::ID start_cart_id = cart_link_iter->first; // should be equal to cart_link_iter->second.links_.id
    TrajectoryPt::ID end_cart_id = cart_link_iter->second.links_.id_next;

    if(!end_cart_id.is_nil())
    {
      std::list<TrajectoryPt::ID> start_joint_ids = cart_link_iter->second.joints_;
      std::list<TrajectoryPt::ID> end_joint_ids = (*cartesian_point_link_)[end_cart_id].joints_;

      if(!calculateEdgeWeights(start_joint_ids, end_joint_ids, edges))
      {
        ROS_WARN("One or more joints lists in the cartesian point link is empty, ID:%s:[start ids: %d], ID:%s:[end ids: %d]",
                boost::uuids::to_string(start_cart_id).c_str(),
                (int)start_joint_ids.size(),
                boost::uuids::to_string(end_cart_id).c_str(),
                (int)end_joint_ids.size());
      }
    }
    else
    {
      // not going to try to calculate from point to nil
      ROS_INFO("Not calculating edge weights to nil ID");
    }
  }

  return !edges.empty();
}

bool PlanningGraph::calculateEdgeWeights(const std::list<TrajectoryPt::ID> &start_joints,const std::list<TrajectoryPt::ID> &end_joints, std::list<JointEdge> &edge_results)
{
  if(start_joints.empty() || end_joints.empty())
  {
    logWarn("One or more joints lists is empty, Start Joints: %i, End Joints: %i",
            start_joints.size(), end_joints.size());
    return false;
  }

  // calculate edges for previous vertices to this set of vertices
  for (std::list<TrajectoryPt::ID>::const_iterator previous_joint_iter = start_joints.begin();
      previous_joint_iter != start_joints.end(); previous_joint_iter++)
  {
    for (std::list<TrajectoryPt::ID>::const_iterator next_joint_iter = end_joints.begin();
        next_joint_iter != end_joints.end(); next_joint_iter++)
    {
      double transition_cost;
      JointTrajectoryPt start_joint = joint_solutions_map_[*previous_joint_iter].first;
      JointTrajectoryPt end_joint = joint_solutions_map_[*next_joint_iter].first;

      transition_cost = linearWeight(start_joint, end_joint);
      JointEdge *edge = new JointEdge();
      edge->joint_start = *previous_joint_iter;
      edge->joint_end = *next_joint_iter;
      edge->transition_cost = transition_cost;

      edge_results.push_back(*edge);
    }
  }

  return edge_results.size() > 0;
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
        vector_diff += fabs(end_vector[i] - start_vector[i]);
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
