/**
 * bdsp_sparse_planner.cpp
 * @brief 
 *
 * @author Jorge Nicho
 * @date Jan 4, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <console_bridge/console.h>

#include <descartes_planner/bdsp_sparse_planner.h>

namespace descartes_planner
{
/**
 *
 * @tparam FloatT
 * @class descartes_planner::UnarySampler: public PointSampler<FloatT>
 * @brief Utility class used by the sparse planner to store a single sample point data computed during the sparse solving stage
 */
template<typename FloatT>
class UnarySampler: public PointSampler<FloatT>
{
public:
  UnarySampler(typename PointData<FloatT>::ConstPtr ref_point):
    ref_point_(ref_point)
  {

  }

  ~UnarySampler()
  {

  }

  /**
   * @brief this method generates the samples
   */
  typename PointSampleGroup<FloatT>::Ptr generate() override
  {
    typename PointSampleGroup<FloatT>::Ptr sample_group = std::make_shared<PointSampleGroup<FloatT>>();
    sample_group->point_id = ref_point_->point_id;
    sample_group->num_samples = 1;
    sample_group->num_dofs = ref_point_->values.size();
    sample_group->values = ref_point_->values;
    return sample_group;
  }

private:
  typename PointData<FloatT>::ConstPtr ref_point_;
};

template<typename FloatT>
class ProxySampler: public PointSampler<FloatT>
{
public:
  ProxySampler(typename PointSampleGroup<FloatT>::Ptr sample_group):
    sample_group_(sample_group)
  {

  }

  ~ProxySampler()
  {

  }

  typename PointSampleGroup<FloatT>::Ptr generate() override
  {
    return sample_group_;
  }

private:
  typename PointSampleGroup<FloatT>::Ptr sample_group_;
};

template<typename FloatT>
BDSPSparsePlanner<FloatT>::BDSPSparsePlanner(typename std::shared_ptr< SamplesContainer<FloatT> > container,
                                             bool report_failures):
  report_failures_(report_failures),
  container_(container),
  graph_planner_(container, report_failures)
{

}

template<typename FloatT>
BDSPSparsePlanner<FloatT>::~BDSPSparsePlanner()
{

}

template<typename FloatT>
bool BDSPSparsePlanner<FloatT>::build(std::vector< typename PointSampler<FloatT>::Ptr >& points,
           FloatT sparse_percentage,
           std::vector<typename EdgeEvaluator<FloatT>::ConstPtr>& edge_evaluators)
{
  if(sparse_percentage >= 1.0 || sparse_percentage <= 0.0)
  {
    throw std::runtime_error("Percentage must be a value in the following range <0,1>");
  }

  std::size_t num_total_points = points.size();
  std::size_t num_selected_points = sparse_percentage * points.size();
  std::vector<std::size_t> selected_indices = {0};
  FloatT interval = num_total_points/num_selected_points;

  std::size_t current_idx = 0;
  std::size_t iter_counter = 0;
  std::size_t last_idx = points.size() - 1;
  while(current_idx < last_idx)
  {
    iter_counter++;
    current_idx = std::floor(iter_counter * interval);
    current_idx = current_idx > last_idx ? last_idx : current_idx;
    selected_indices.push_back(current_idx);
  }

  return build(points, selected_indices, edge_evaluators);
}

template<typename FloatT>
bool BDSPSparsePlanner<FloatT>::build(std::vector< typename PointSampler<FloatT>::Ptr >& points,
           std::vector<std::size_t> selected_sparse_points_indices,
           std::vector<typename EdgeEvaluator<FloatT>::ConstPtr>& edge_evaluators)
{

  // clearing arrays
  failed_points_.clear();
  failed_edges_.clear();

  // get selected points and edges first
  std::vector< typename PointSampler<FloatT>::Ptr > selected_sparse_points;
  std::vector<typename EdgeEvaluator<FloatT>::ConstPtr> selected_sparsed_edge_evaluators;
  for(const auto& idx : selected_sparse_points_indices)
  {
    selected_sparse_points.push_back(points[idx]);
  }

  for(std::size_t i = 0; i < selected_sparse_points_indices.size() - 1; i++)
  {
    std::size_t idx = selected_sparse_points_indices[i];
    selected_sparsed_edge_evaluators.push_back(edge_evaluators[idx]);
  }

  // build and solve for selected points now
  std::vector< typename PointData<FloatT>::ConstPtr > sparse_solution_points;
  {
    BDSPGraphPlanner<FloatT> graph_planner(container_, report_failures_);
    if(!graph_planner.build(selected_sparse_points, selected_sparsed_edge_evaluators ))
    {
      CONSOLE_BRIDGE_logError("Failed to build graph for selected points");
      return false;
    }

    if(!graph_planner.solve(sparse_solution_points))
    {
      CONSOLE_BRIDGE_logError("Failed to solve graph for selected points");
      return false;
    }
  }
  CONSOLE_BRIDGE_logInform("Found sparse solution with %lu points", selected_sparse_points.size());

  // now build graph for sparse and intermediate points
  std::vector< typename PointSampler<FloatT>::Ptr > dense_point_samplers;
  dense_point_samplers.reserve(points.size());
  typename PointSampler<FloatT>::Ptr sampler_0;
  typename PointSampler<FloatT>::Ptr sampler_f;
  for(std::size_t i = 1; i < selected_sparse_points_indices.size(); i++)
  {
    std::size_t p0_idx = selected_sparse_points_indices[i - 1];
    std::size_t pf_idx = selected_sparse_points_indices[i];

    // initial and final sampler for this segment only return a single point sample
    typename PointData<FloatT>::ConstPtr point_data_0 = sparse_solution_points[i-1];
    typename PointData<FloatT>::ConstPtr point_data_f = sparse_solution_points[i];
    sampler_0 = std::make_shared<UnarySampler<FloatT>>(point_data_0);
    sampler_f = std::make_shared<UnarySampler<FloatT>>(point_data_f);
    dense_point_samplers.push_back(sampler_0);

    if(pf_idx - p0_idx <= 1 )
    {
      // no intermediate points in between initial and final, go to next segment, end sampler will be added on next iter
      continue;
    }

    // get samples from each intermediate point
    FloatT t;
    std::size_t segment_length = pf_idx - p0_idx;
    for(std::size_t ii = p0_idx + 1 ; ii < pf_idx; ii++)
    {
      t = (ii - p0_idx)/segment_length;
      typename PointSampler<FloatT>::Ptr intermediate_sampler = points[ii];
      auto interpolated_point_data = point_data_0->interpolate(t, point_data_f);
      typename PointSampleGroup<FloatT>::Ptr closest_sample_group = intermediate_sampler->getClosest(interpolated_point_data);

      if(!closest_sample_group)
      {
        if(report_failures_)
        {
          failed_points_.push_back(i);
          continue;
        }

        CONSOLE_BRIDGE_logError("Failed to generate closest samples for intermediate point %lu",ii);
        return false;
      }
      // create proxy sampler that just returns the closest samples
      typename PointSampler<FloatT>::Ptr proxy_sampler = std::make_shared<ProxySampler<FloatT>>(closest_sample_group);
      dense_point_samplers.push_back(proxy_sampler);
    }
  }

  // adding final point in segment
  dense_point_samplers.push_back(sampler_f);

  if(!failed_points_.empty())
  {
    return false;
  }
  CONSOLE_BRIDGE_logInform("Dense interpolated trajectory has %lu points, original had %lu points",
                           dense_point_samplers.size(), points.size());

  // build graph with all points now
  failed_points_.clear();
  failed_edges_.clear();
  bool succeeded = graph_planner_.build(dense_point_samplers, edge_evaluators);
  if(!succeeded && report_failures_)
  {
    graph_planner_.getFailedPoints(failed_points_);
    graph_planner_.getFailedEdges(failed_edges_);
  }

  return succeeded;
}

template<typename FloatT>
bool BDSPSparsePlanner<FloatT>::solve(std::vector< typename PointData<FloatT>::ConstPtr >& solution_points)
{
  return graph_planner_.solve(solution_points);
}

template<typename FloatT>
bool BDSPSparsePlanner<FloatT>::getFailedEdges(std::vector<std::size_t>& failed_edges)
{
  failed_edges = failed_edges_;
  return report_failures_;
}

template<typename FloatT>
bool BDSPSparsePlanner<FloatT>::getFailedPoints(std::vector<std::size_t>& failed_points)
{
  failed_points = failed_points_;
  return report_failures_;
}

// explicit specializations
template class BDSPSparsePlanner<float>;
template class BDSPSparsePlanner<double>;

} /* namespace descartes_planner */
