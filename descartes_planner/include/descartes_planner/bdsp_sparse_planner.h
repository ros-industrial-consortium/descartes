/**
 * bdsp_sparse_planner.h
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

#ifndef INCLUDE_DESCARTES_PLANNER_BDSP_SPARSE_PLANNER_H_
#define INCLUDE_DESCARTES_PLANNER_BDSP_SPARSE_PLANNER_H_

#include "descartes_planner/common.h"
#include "descartes_planner/bdsp_graph_planner.h"

namespace descartes_planner
{

template <typename FloatT = float>
class BDSPSparsePlanner
{
public:
  BDSPSparsePlanner(typename std::shared_ptr< SamplesContainer<FloatT> > container = nullptr,
                    bool report_failures = false);
  virtual ~BDSPSparsePlanner();

  bool build(std::vector< typename PointSampler<FloatT>::Ptr >& points,
             float selected_percentage,
             std::vector<typename EdgeEvaluator<FloatT>::ConstPtr>& edge_evaluators);

  /**
   * @brief Builds the graph
   * @param points          A vector of point samplers
   * @param selected_point_indices
   * @param edge_evaluator  A vector of speed evaluators, the vector size should be one less than the point samplers vector
   * @return True on success false otherwise
   */
  bool build(std::vector< typename PointSampler<FloatT>::Ptr >& points,
             std::vector<std::size_t> selected_point_indices,
             std::vector<typename EdgeEvaluator<FloatT>::ConstPtr>& edge_evaluators);

  /**
   * @brief solves the plan by searching for the lowest cost solution, use only after calling the build method
   * @param solution_points  The solution
   * @return True on success, false otherwise
   */
  bool solve(std::vector< typename PointData<FloatT>::ConstPtr >& solution_points);

  bool getFailedEdges(std::vector<std::size_t>& failed_edges);
  bool getFailedPoints(std::vector<std::size_t>& failed_points);

private:

  std::vector< typename EdgeEvaluator<FloatT>::ConstPtr > edge_evaluators_;
  BDSPGraphPlanner<FloatT> graph_planner_;
  typename std::shared_ptr< SamplesContainer<FloatT> > container_;
  bool report_failures_;
  std::vector<std::size_t> failed_points_;
  std::vector<std::size_t> failed_edges_;

};



} /* namespace descartes_planner */

#endif /* INCLUDE_DESCARTES_PLANNER_BDSP_SPARSE_PLANNER_H_ */
