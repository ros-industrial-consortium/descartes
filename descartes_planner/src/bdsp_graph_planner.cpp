/*
 * graph_solver.cpp
 *
 *  Created on: Aug 30, 2019
 *      Author: Jorge Nicho
 */

#include <numeric>

#include <fstream>

#include <memory>

#include <console_bridge/console.h>

#include <boost/format.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/dijkstra_shortest_paths_no_color_map.hpp>

#include "descartes_planner/bdsp_graph_planner.h"

static const int VIRTUAL_VERTEX_INDEX = -1;

namespace descartes_planner
{

template<typename FloatT>
descartes_planner::BDSPGraphPlanner<FloatT>::BDSPGraphPlanner(typename std::shared_ptr< SamplesContainer<FloatT> > container,
                                                    bool report_failures):
  container_(container),
  report_failures_(report_failures)
{
  if(container_ == nullptr)
  {
    // if no container is provided then use default implementation
    container_ = std::make_shared< DefaultSamplesContainer<FloatT> >();
  }
}

template<typename FloatT>
descartes_planner::BDSPGraphPlanner<FloatT>::~BDSPGraphPlanner()
{

}

template<typename FloatT>
typename EdgeEvaluator<FloatT>::ConstPtr descartes_planner::BDSPGraphPlanner<FloatT>::getEdgeEvaluator(std::uint32_t idx)
{
  if(edge_evaluators_.size() == 1)
  {
    return edge_evaluators_.front();
  }
  else
  {
    return edge_evaluators_.at(idx);
  }
}

template<typename FloatT>
void descartes_planner::BDSPGraphPlanner<FloatT>::setup(std::vector< typename PointSampler<FloatT>::Ptr >& points,
                                                   std::vector<typename EdgeEvaluator<FloatT>::ConstPtr>& edge_evaluators)
{

  failed_points_.clear();
  failed_edges_.clear();

  // setting up point sampler container
  points_.clear();
  std::copy(points.begin(),points.end(),std::back_inserter(points_));
  container_->clear();
  container_->allocate(points_.size());

  // setting up edge evaluators
  edge_evaluators_.clear();
  if(edge_evaluators.size() == 1)
  {
    edge_evaluators_ = edge_evaluators;
  }
  else if(edge_evaluators.size() == points_.size() - 1)
  {
    std::copy(edge_evaluators.begin(),edge_evaluators.end(),std::back_inserter(edge_evaluators_));
  }
  else if(edge_evaluators.empty())
  {
    throw std::runtime_error("Edge evaluators vector is empty");
  }
  else if(edge_evaluators.size() > 1 && edge_evaluators.size() != points.size() - 1)
  {
    throw std::runtime_error(boost::str(
        boost::format("Edge evaluators vector's size (%lu) must be one less than that of the points vector (%lu)") %
        edge_evaluators.size() % points.size() ) );
  }
}

template<typename FloatT>
bool descartes_planner::BDSPGraphPlanner<FloatT>::build(std::vector< typename PointSampler<FloatT>::Ptr >& points,
           typename EdgeEvaluator<FloatT>::ConstPtr edge_evaluator)
{
  // setting up edge evaluators
  std::vector<typename EdgeEvaluator<FloatT>::ConstPtr> edge_evaluators = {edge_evaluator};
  return build(points, edge_evaluators);
}

template<typename FloatT>
std::vector< EdgeProperties<FloatT> > descartes_planner::BDSPGraphPlanner<FloatT>::filterDisconnectedEdges(
    const std::vector< EdgeProperties<FloatT> >& edges,const std::map<int, VertexProperties>& connected_src_vertices,
    std::uint32_t current_vertex_count) const
{
  auto new_edges = edges;
  auto start_loc = new_edges.begin();
  auto end_loc = new_edges.end();
  auto new_end_loc = std::remove_if(new_edges.begin(), new_edges.end(), [&](const EdgeProperties<FloatT>& edge){
    int src_idx = edge.src_vtx.sample_index + current_vertex_count;
    return connected_src_vertices.count(src_idx) == 0;
  });

  new_edges.erase(new_end_loc, end_loc);
  return new_edges;
}

template<typename FloatT>
bool descartes_planner::BDSPGraphPlanner<FloatT>::build(std::vector<typename PointSampler<FloatT>::Ptr>& points,
                                                   std::vector<typename EdgeEvaluator<FloatT>::ConstPtr>& edge_evaluators)
{
  setup(points, edge_evaluators);

  //// adding virtual vertex
  graph_.clear();

  // generating samples now
  for(std::size_t  i = 0; i < points_.size(); i++)
  {
    typename PointSampleGroup<FloatT>::Ptr samples = points_[i]->generate();
    if(!samples || samples->values.empty())
    {
      CONSOLE_BRIDGE_logError("Failed to generate samples for point %lu",i);
      if(report_failures_)
      {
        failed_points_.push_back(i);
        continue;
      }
      return false;
    }
    container_->at(i) = samples;
  }

  // no need to proceed if sample generation failed
  if(!failed_points_.empty())
  {
    CONSOLE_BRIDGE_logError("Failed to generate one or more point samples, use getFailedPoints to get the failed points");
    return false;
  }

  // build the graph now
  typename PointSampleGroup<FloatT>::Ptr samples1 = nullptr;
  typename PointSampleGroup<FloatT>::Ptr samples2 = nullptr;

  std::uint32_t vertex_count = 1;
  bool add_virtual_vertex = true;
  std::map<int, VertexProperties> src_vertices_added;
  std::map<int, VertexProperties> dst_vertices_added;

  // use samples to populate edges in order to build the search graph
  for(std::size_t i = 1; i < points_.size(); i++)
  {
    std::size_t p1_idx = i -1;
    std::size_t p2_idx = i;

    samples1 = (*container_)[p1_idx];
    samples2 = (*container_)[p2_idx];
    samples1->point_id = p1_idx; // TODO: setting ids may not be necessary
    samples2->point_id = p2_idx;

    // validating vertex samples
    using SampleMap = std::map< std::size_t, typename PointSampleGroup<FloatT>::Ptr >;
    SampleMap sample_groups = {{p1_idx, samples1}, {p2_idx, samples2}};
    if(!std::all_of(sample_groups.begin(), sample_groups.end(),[](typename SampleMap::value_type& kv){
      if(kv.second == nullptr)
      {
        CONSOLE_BRIDGE_logError("Invalid samples received for point with index %lu",kv.first);
        return false;
      }
      if(kv.second->values.empty())
      {
        CONSOLE_BRIDGE_logError("No valid samples were found in point %lu",kv.first);
        return false;
      }
      return true;
    }))
    {
      return false;
    }

    // evaluate edges
    using EdgeProp = EdgeProperties<FloatT>;
    auto edge_evaluator = getEdgeEvaluator(p1_idx);
    std::vector< EdgeProperties<FloatT> > edges = edge_evaluator->evaluate(samples1, samples2);

    if(edges.empty())
    {
      CONSOLE_BRIDGE_logError("Edge evaluation between points %lu and %lu failed", samples1->point_id,
                              samples2->point_id);
      if(report_failures_)
      {
        failed_edges_.push_back(p1_idx);
        continue;
      }
      return false;
    }

    // no need to proceed if an edge has already failed
    if(!failed_edges_.empty())
    {
      continue;
    }

    CONSOLE_BRIDGE_logDebug("Found %lu edges between nodes (%i, %i)",edges.size(),samples1->point_id ,samples2->point_id );

    // check that at least one is valid
    std::size_t num_valid_edges = std::accumulate(edges.begin(), edges.end(),0,[](std::size_t c, const EdgeProperties<FloatT>& edge){
      return c + (edge.valid ? 1 : 0);
    });
    if(num_valid_edges == 0)
    {
      CONSOLE_BRIDGE_logError("Not a single valid edge was found between points (%lu, %lu)",p1_idx,p2_idx);
      return false;
    }
    else
    {
      CONSOLE_BRIDGE_logDebug("Point (%lu, %lu) has %lu valid edges out of %lu = %i x %i",p1_idx,p2_idx,num_valid_edges,edges.size(),
                               samples1->num_samples, samples2->num_samples);
    }

    // filtering disconnected edges
    if(!dst_vertices_added.empty())
    {
      edges = filterDisconnectedEdges(edges, dst_vertices_added, vertex_count);
      if(edges.empty())
      {
        CONSOLE_BRIDGE_logError("Edge between points %lu and %lu has no continuous path", samples1->point_id,
                                samples2->point_id);
        if(report_failures_)
        {
          failed_edges_.push_back(p1_idx);
          continue;
        }
        return false;
      }
    }

    src_vertices_added.clear();
    dst_vertices_added.clear();
    for(EdgeProp& edge: edges)
    {
      if(!edge.valid)
      {
        continue;
      }

      if(edge.weight >= std::numeric_limits<FloatT>::max())
      {
        CONSOLE_BRIDGE_logError("Found edge with very high weight value between points (%lu, %lu)", p1_idx, p2_idx);
        continue;
      }

      bool added;

      std::uint32_t src_vtx_index = edge.src_vtx.sample_index + vertex_count;
      std::uint32_t dst_vtx_index =  edge.dst_vtx.sample_index + vertex_count + samples1->num_samples;

      if(src_vtx_index >= dst_vtx_index)
      {
        CONSOLE_BRIDGE_logError("Found equal source and destination vertices values at iteration", i);
        return false;
      }

      if(src_vtx_index <= 0)
      {
        CONSOLE_BRIDGE_logError("Source vertex is negative at iteration %i", i);
        return false;
      }

      // adding edge to virtual vertex first
      if(add_virtual_vertex && (src_vertices_added.count(src_vtx_index) == 0))
      {
        typename GraphT::edge_descriptor e;


        CONSOLE_BRIDGE_logDebug("Adding edge (0, %lu) to virtual vertex",src_vtx_index);
        VertexProperties virtual_vertex_props;
        virtual_vertex_props.point_id = VIRTUAL_VERTEX_INDEX;
        virtual_vertex_props.sample_index = 0;
        EdgeProperties<FloatT> virtual_edge= { .weight = 0, .valid = edge.valid,
                                               .src_vtx = virtual_vertex_props, .dst_vtx = edge.src_vtx};
        boost::tie(e,added) = boost::add_edge(0, src_vtx_index, graph_);
        if(!added)
        {
          CONSOLE_BRIDGE_logWarn("Edge (%lu, %lu) has already been added to the graphs",0,
                                 src_vtx_index);
          return false;
        }
        graph_[e] = virtual_edge;

      }

      typename GraphT::edge_descriptor e;
      boost::tie(e,added) = boost::add_edge(src_vtx_index, dst_vtx_index, graph_);
      CONSOLE_BRIDGE_logDebug("Added edge (%lu, %lu)",src_vtx_index, dst_vtx_index);
      if(!added)
      {
        CONSOLE_BRIDGE_logError("Edge (%lu, %lu) has already been added to the graphs",src_vtx_index,
                                dst_vtx_index);
        return false;
      }
      else
      {
        // setting edge properties
        graph_[e]= edge;
      }

      src_vertices_added[src_vtx_index] = edge.src_vtx;
      dst_vertices_added[dst_vtx_index] = edge.dst_vtx;
    }

    if(src_vertices_added.empty() || dst_vertices_added.empty())
    {
      CONSOLE_BRIDGE_logError("No continuous path could be found between points %i and %i",p1_idx, p2_idx);
      return false;
    }

    vertex_count += samples1->num_samples;
    add_virtual_vertex = false; // do not add edges for the virtual vertex anymore

  }

  if(!failed_edges_.empty())
  {
    CONSOLE_BRIDGE_logError("Failed to generate one or more point samples, use getFailedEdges to get the failed edges");
    return false;
  }

  end_vertices_ = dst_vertices_added;
  return true;
}

template<typename FloatT>
bool descartes_planner::BDSPGraphPlanner<FloatT>::getFailedEdges(std::vector<std::size_t>& failed_edges)
{
 if(!report_failures_)
 {
   CONSOLE_BRIDGE_logError("Planner was not configured to report failures");
   return false;
 }

 failed_edges = failed_edges_;
 return true;
}

template<typename FloatT>
bool descartes_planner::BDSPGraphPlanner<FloatT>::getFailedPoints(std::vector<std::size_t>& failed_points)
{
  if(!report_failures_)
  {
    CONSOLE_BRIDGE_logError("Planner was not configured to report failures");
    return false;
  }

  failed_points = failed_points_;
  return true;
}

template<typename FloatT>
void descartes_planner::BDSPGraphPlanner<FloatT>::writeGraphLogs(const std::vector<FloatT>& weights,
                                                               const std::vector<typename GraphT::vertex_descriptor>& predecessors)
{
  typedef boost::graph_traits<GraphT> GraphTraits;

  // declare file names
  const std::string dot_file_name = "dijkstra-no-color-map-eg.dot";
  const std::string txt_file_name = "dijkstra_shortest_path_dump.txt";

  // writing dot file
  std::ofstream dot_file(dot_file_name);

  dot_file << "digraph D {\n"
    << "  rankdir=LR\n"
    << "  size=\"4,3\"\n"
    << "  ratio=\"fill\"\n"
    << "  edge[style=\"bold\"]\n" << "  node[shape=\"circle\"]\n";

  typename GraphTraits::edge_iterator  ei, ei_end;
  for (boost::tie(ei, ei_end) = edges(graph_); ei != ei_end; ++ei) {
    typename GraphTraits::edge_descriptor e = *ei;
    typename GraphTraits::vertex_descriptor u = source(e, graph_), v = target(e, graph_);
    EdgeProperties<FloatT> edge_props = graph_[e];
    VertexProperties u_vertex = edge_props.src_vtx;
    VertexProperties v_vertex = edge_props.dst_vtx;

    if(u_vertex.point_id < 0 || v_vertex.point_id < 0)
    {
      continue;
    }

    dot_file << "\tP" << u_vertex.point_id << "_S" << u_vertex.sample_index <<"_G" << u << " -> ";
    dot_file << "P" << v_vertex.point_id << "_S" << v_vertex.sample_index <<"_G" << v;

    //dot_file << u << " -> " << v;
    dot_file << " [label=\"" << int(1e3 * edge_props.weight)  << "\"";
    if (predecessors[v] == u)
      dot_file << ", color=\"black\"";
    else
      dot_file << ", color=\"red\"";
    dot_file << "];\n";
  }
  dot_file << "}";

  // writing text file
  std::ofstream graph_file(txt_file_name);
  std::ostream& ref = graph_file;
  boost::print_graph(graph_, ref);
  graph_file.close();

  // print log names
  const auto log_file_names = {dot_file_name, txt_file_name};
  for(const auto& f : log_file_names)
  {
    CONSOLE_BRIDGE_logInform("wrote graph log file %s", f.c_str());
  }
}

template<typename FloatT>
bool descartes_planner::BDSPGraphPlanner<FloatT>::solve(
    std::vector<typename PointData<FloatT>::ConstPtr>& solution_points)
{
  typename GraphT::vertex_descriptor virtual_vertex = vertex(0, graph_), current_vertex;
  std::size_t num_vert = boost::num_vertices(graph_);
  std::vector<typename GraphT::vertex_descriptor> predecessors(num_vert);
  std::vector<FloatT> weights(num_vert, 0.0);

/*  boost::dijkstra_shortest_paths(graph_, virtual_vertex,
    weight_map(get(&EdgeProperties<FloatT>::weight, graph_))
    .distance_map(boost::make_iterator_property_map(weights.begin(),get(boost::vertex_index, graph_)))
    .predecessor_map(&predecessors[0]));*/

  boost::dijkstra_shortest_paths_no_color_map(graph_, virtual_vertex,
   weight_map(get(&EdgeProperties<FloatT>::weight, graph_))
   .distance_map(boost::make_iterator_property_map(weights.begin(),get(boost::vertex_index, graph_)))
   .predecessor_map(boost::make_iterator_property_map(predecessors.begin(),get(boost::vertex_index, graph_))));

  CONSOLE_BRIDGE_logDebug("Num vertices %i", num_vert);
  CONSOLE_BRIDGE_logDebug("Predecessor array size %lu",predecessors.size());
  CONSOLE_BRIDGE_logDebug("Weights array size %lu",weights.size());
  CONSOLE_BRIDGE_logDebug("End vertices size %lu", end_vertices_.size());

  // iterating through out edges while inspecting the predecessor
  typedef boost::graph_traits<GraphT> GraphTraits;
  typename GraphT::vertex_descriptor cheapest_end_vertex = -1;
  double cost = std::numeric_limits<FloatT>::infinity();

  for(std::map<int, VertexProperties>::value_type& kv: end_vertices_)
  {
    typename GraphT::vertex_descriptor candidate_vertex = kv.first;
    CONSOLE_BRIDGE_logDebug("Searching end vertex %i with cost %f", candidate_vertex,
                           weights[candidate_vertex]);
    if(weights[candidate_vertex] > cost)
    {
      CONSOLE_BRIDGE_logDebug("cost too high, skipping to next end vertex");
      continue;
    }
    cost = weights[candidate_vertex];

    // check if it is connected
    typename GraphT::vertex_descriptor prev_vertex = predecessors[candidate_vertex];
    typename GraphTraits::out_edge_iterator out_i, out_end;
    for(boost::tie(out_i, out_end)= boost::out_edges(prev_vertex, graph_); out_i != out_end; out_i++)
    {
      typename GraphTraits::edge_descriptor e = *out_i;
      typename GraphT::vertex_descriptor targ = boost::target(e, graph_);
      if(targ == candidate_vertex)
      {
        cheapest_end_vertex = candidate_vertex;
        break;
      }
    }
  }
  current_vertex = cheapest_end_vertex;
  if(static_cast<int>(current_vertex) < 0 )
  {
    CONSOLE_BRIDGE_logError("Found no continuous solution path through graph");
    writeGraphLogs(weights, predecessors);
    return false;
  }

  CONSOLE_BRIDGE_logInform("Found valid shortest path with end vertex: %i and cost %f", current_vertex, cost);

  solution_points.resize(container_->size(), nullptr);
  auto add_solution = [&](VertexProperties& vp) -> bool{

    if(vp.point_id  == VIRTUAL_VERTEX_INDEX)
    {
      // found virtual index, just return
      return true;
    }

    if(vp.point_id >= points_.size())
    {
      CONSOLE_BRIDGE_logError("Source vertex index %i exceeds point buffer of size %lu",vp.point_id, points_.size());
      return false;
    }

    typename PointSampleGroup<FloatT>::Ptr  sample_group = container_->at(vp.point_id);

    // recompute or retrieve the sample and storing it
    if(solution_points[vp.point_id] != nullptr) // can not have more than one solutions
    {
      CONSOLE_BRIDGE_logDebug("Sample for point %i has already been assigned", vp.point_id);
      return true;
    }

    typename PointData<FloatT>::ConstPtr point_data = sample_group->at(vp.sample_index);
    if(!point_data)
    {
      CONSOLE_BRIDGE_logError("SampleGroup %i has no sample %lu", vp.point_id, vp.sample_index);
      return false;
    }
    solution_points[vp.point_id] = point_data;
    CONSOLE_BRIDGE_logDebug("Added %s solution point %i of %lu points",(point_data != nullptr ? "valid" : "null"),
                             vp.point_id, solution_points.size());
    return true;
  };

  int vertex_counter = 0;
  while(current_vertex != virtual_vertex)
  {
    typename GraphT::vertex_descriptor prev_vertex = predecessors[current_vertex];
    typename GraphTraits::out_edge_iterator out_i, out_end;
    bool found_next = false;
    for(boost::tie(out_i, out_end)= boost::out_edges(prev_vertex, graph_); out_i != out_end; out_i++)
    {
      typename GraphTraits::edge_descriptor e = *out_i;
      typename GraphT::vertex_descriptor targ = boost::target(e, graph_);
      if(targ == current_vertex)
      {
        found_next = true;
        current_vertex = prev_vertex;

        // grab sampler
        EdgeProperties<FloatT> edge_props = graph_[e];
        CONSOLE_BRIDGE_logDebug("Points %lu and %lu connected by edge (%lu, %lu)",
                                 edge_props.src_vtx.point_id, edge_props.dst_vtx.point_id,
                                 prev_vertex, targ);

        if( !(add_solution(edge_props.dst_vtx) && add_solution(edge_props.src_vtx)))
        {
          break;
        }
      }
    }

    if(!found_next )
    {
      break;
    }

    vertex_counter++;
  }

  CONSOLE_BRIDGE_logDebug("Exiting vertex traversing loop with vertex count at %i", vertex_counter);

  for(std::size_t i = 0; i < solution_points.size(); i++)
  {
    typename PointData<FloatT>::ConstPtr point_data = solution_points[i];
    if(point_data == nullptr)
    {
      CONSOLE_BRIDGE_logError("Invalid solution for point %lu was found",i);
      return false;
    }
  }
  return true;
}

// explicit specializations
template class BDSPGraphPlanner<float>;
template class BDSPGraphPlanner<double>;

} /* namespace descartes_planner */
