/*
 * graph_solver.cpp
 *
 *  Created on: Aug 30, 2019
 *      Author: jrgnicho
 */

#include <console_bridge/console.h>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include "descartes_planner/graph_solver.h"

namespace descartes_planner
{

// explicit especializations
template class GraphSolver<float>;
template class GraphSolver<double>;

template<typename FloatT>
descartes_planner::GraphSolver<FloatT>::GraphSolver(typename EdgeEvaluator<FloatT>::ConstPtr edge_evaluator):
  edge_evaluator_(edge_evaluator)
{

}

template<typename FloatT>
descartes_planner::GraphSolver<FloatT>::~GraphSolver()
{

}

template<typename FloatT>
bool descartes_planner::GraphSolver<FloatT>::build(std::vector<typename PointSampler<FloatT>::Ptr>& points)
{
  points_.clear();
  std::copy(points.begin(),points.end(),std::back_inserter(points_));

  // build the graph now
  typename PointSampleGroup<FloatT>::Ptr samples1 = nullptr;
  typename PointSampleGroup<FloatT>::Ptr samples2 = nullptr;

  auto gather_samples = [](typename PointSampleGroup<FloatT>::Ptr samples,typename  PointSampler<FloatT>::Ptr sampler)
  {
    if(samples == nullptr)
    {
      samples = sampler->getSamples();
      return;
    }

    // preallocating
    samples->num_samples = sampler->getNumSamples();
    samples->num_dofs = sampler->getDofs();
    if(samples->values.size() < samples->num_samples * samples->num_dofs)
    {
      samples->values.resize(samples->num_samples * samples->num_dofs);
    }

    sampler->getSamples(samples);
  };

  std::size_t vertex_count = 1; // starting at 1 to account for initial virtual vertex
  for(std::size_t i = 1; i < points_.size(); i++)
  {
    std::size_t p1_idx = i -1;
    std::size_t p2_idx = i;

    typename PointSampler<FloatT>::Ptr sampler1 = points_[p1_idx];
    typename PointSampler<FloatT>::Ptr sampler2 = points_[p2_idx];

    // reuse samples of previous point if they were created
    if(samples2 != nullptr)
    {
      samples1 = samples2;
    }
    else
    {
      gather_samples(samples1,sampler1);
    }

    // always recompute samples for the next point
    gather_samples(samples2,sampler2);

    // validating vertex samples
    using SampleMap = std::map< std::size_t, typename PointSampleGroup<FloatT>::Ptr >;
    SampleMap sample_groups = {{p1_idx, samples1}, {p2_idx, samples2}};
    if(!std::all_of(sample_groups.begin(), sample_groups.end(),[](typename SampleMap::value_type& kv){

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
    std::vector< EdgeProperties<FloatT> > edges = edge_evaluator_->evaluate(samples1, samples2);
    for(EdgeProp& edge: edges)
    {
      typename GraphT::edge_descriptor e;
      bool added;
      if(edge.valid)
      {
        std::size_t src_vtx_index = edge.src_vtx.sample_index + vertex_count;
        std::size_t dst_vtx_index = samples1->num_samples + edge.dst_vtx.sample_index + vertex_count;
        boost::tie(e,added) = boost::add_edge(src_vtx_index, dst_vtx_index, graph_);
        if(!added)
        {
          CONSOLE_BRIDGE_logError("Edge (%lu, %lu) has already been added to the graphs",src_vtx_index,
                                  dst_vtx_index);
          return false;
        }

        // setting vertex properties
        graph_[e] = edge;

        // incrementing counter
        vertex_count += samples1->num_samples + samples2->num_samples;

        if(i == 1)
        {
          VertexProperties virtual_vertex_props;
          virtual_vertex_props.point_id = -1;
          virtual_vertex_props.sample_index = 0;
          EdgeProperties<FloatT> virtual_edge= {.src_vtx = virtual_vertex_props, .dst_vtx = edge.dst_vtx,
            .valid = true, .weight = 0.0};
          boost::tie(e,added) = boost::add_edge(0, src_vtx_index, graph_);
          graph_[e] = virtual_edge;
        }
      }
    }
  }
}

template<typename FloatT>
bool descartes_planner::GraphSolver<FloatT>::solve(
    std::vector<typename PointSampleGroup<FloatT>::ConstPtr>& solution_points)
{
  typename GraphT::vertex_descriptor virtual_vertex = vertex(0, graph_), current_vertex;
  std::size_t num_vert = boost::num_vertices(graph_);
  std::vector<typename GraphT::vertex_descriptor> predecessors(num_vert);
  std::vector<FloatT> weights(num_vert, std::numeric_limits<FloatT>::max());

  boost::dijkstra_shortest_paths(graph_, virtual_vertex,
    weight_map(get(&EdgeProperties<FloatT>::weight, graph_))
    .distance_map(boost::make_iterator_property_map(weights.begin(),get(boost::vertex_index, graph_)))
    .predecessor_map(&predecessors[0]));

  // iterating through out edges while inspecting the predecessor
  typedef boost::graph_traits<GraphT> GraphTraits;
  typename GraphTraits::out_edge_iterator out_i, out_end;
  current_vertex = virtual_vertex;
  solution_points.resize(points_.size(), nullptr);
  std::size_t points_added = 0;
  bool found_next = false;
  do
  {
    found_next = false;
    for(boost::tie(out_i, out_end)= boost::out_edges(current_vertex, graph_); out_i != out_end; out_i++)
    {
      typename GraphTraits::edge_descriptor e = *out_i;
      typename GraphT::vertex_descriptor targ = boost::target(e, graph_);
      if(predecessors[targ] == current_vertex)
      {
        current_vertex = targ;

        // grab sampler
        EdgeProperties<FloatT> edge_props = graph_[e];
        if(edge_props.src_vtx.point_id >= points_.size())
        {
          CONSOLE_BRIDGE_logError("Source vertex index %lu is out of range",edge_props.src_vtx.point_id);
          break;
        }
        typename PointSampler<FloatT>::Ptr  sampler = points_[edge_props.src_vtx.point_id];

        // recompute or retrieve the sample and storing it
        if(solution_points[edge_props.src_vtx.point_id] != nullptr) // can not have more than one solutions
        {
          CONSOLE_BRIDGE_logError("Sample for point %i has already been assigned", edge_props.src_vtx.point_id);
          break;
        }
        typename PointSampleGroup<FloatT>::Ptr sample = sampler->getSample(edge_props.src_vtx.sample_index);
        solution_points[edge_props.src_vtx.point_id] = sample;
        found_next = true;
        break;
      }
    }
  } while(found_next);

  bool all_valid = std::all_of(solution_points.begin(), solution_points.end(),
                               [](typename PointSampleGroup<FloatT>::ConstPtr sample){
    return sample != nullptr;
  });
  return all_valid;
}

} /* namespace descartes_planner */
