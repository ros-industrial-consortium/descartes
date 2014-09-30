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
//#include "descartes_path_planning/Planning_path.h"

namespace descartes
{

struct PlanningGraphState
{
  int state_id;    // Reference to State object in Planning path
};

struct PlanningGraphTransition
{
  double transition_cost;
  double total_edge_cost;
};

typedef PlanningGraphState Vertex;
typedef PlanningGraphTransition Edge;

typedef boost::adjacency_list< boost::listS,            /*edge container*/
                               boost::vecS,             /*vertex_container*/
                               boost::directedS,        /*graph type*/
                               Vertex,                  /*vertex structure*/
                               Edge,                    /*edge structure*/
                               boost::no_property,      /*graph property?*/
                               boost::listS             /*edge container (not used for directed graphs)*/
                             > PlanningGraph;
typedef PlanningGraph Graph;     // TODO this can probably go away and we will call everything PlanningGraph
typedef boost::graph_traits<Graph>::vertex_descriptor   VertexDescriptor;
typedef boost::graph_traits<Graph>::vertex_iterator     VertexIterator;
typedef boost::graph_traits<Graph>::edge_descriptor     EdgeDescriptor;
typedef boost::graph_traits<Graph>::edge_iterator       EdgeIterator;
typedef boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;
typedef boost::property_map<Graph, boost::vertex_index_t>::const_type IndexMapC;
//typedef boost::property_map<Graph, int>::type           PlanningStateIdMap;
typedef std::map<size_t, std::set<VertexDescriptor> > PlanningPtIdMap;   /**<All vertices that correspond to a particular pt in Planning path*/
//TODO should this be a path iterator?

class PlanningGraph: public Graph
{
public:
  PlanningGraph() {};
  virtual ~PlanningGraph() {};
};

} /* namespace descartes */
#endif /* PLANNING_GRAPH_H_ */
