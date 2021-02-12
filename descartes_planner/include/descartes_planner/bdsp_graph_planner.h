/*
 * graph_solver.h
 *
 *  Created on: Aug 30, 2019
 *      Author: jrgnicho
 */

#ifndef INCLUDE_DESCARTES_PLANNER_GRAPH_SOLVER_H_
#define INCLUDE_DESCARTES_PLANNER_GRAPH_SOLVER_H_

#include <memory>
#include <boost/graph/adjacency_list.hpp>
#include "descartes_planner/common.h"
namespace descartes_planner
{

/**
 * @class DefaultSamplesContainer
 * @brief Default Sample container implementation
 * @tparam FloatT
 */
template <typename FloatT = float>
class DefaultSamplesContainer: public SamplesContainer<FloatT>
{
public:
  DefaultSamplesContainer(){}
  virtual ~DefaultSamplesContainer(){ }

  /**
   * @brief this method should set the size of the internal buffer and clear previous data
   * @param n
   */
  void allocate(std::size_t n) override
  {
    sample_groups_.resize(n, nullptr);
  }

  void clear() override
  {
    sample_groups_.clear();
  }

  bool has(std::size_t idx) override
  {
    return sample_groups_.size() > idx && sample_groups_[idx] != nullptr;
  }

  std::size_t size() override
  {
    return sample_groups_.size();
  }

  typename PointSampleGroup<FloatT>::Ptr& at(std::size_t idx) override
  {
    return sample_groups_.at(idx);
  }

  const typename PointSampleGroup<FloatT>::Ptr& at(std::size_t idx) const override
  {
    return sample_groups_.at(idx);
  }

  typename PointSampleGroup<FloatT>::Ptr& operator[](std::size_t idx) override
  {
    return sample_groups_[idx];
  }

  const typename PointSampleGroup<FloatT>::Ptr& operator[](std::size_t idx) const override
  {
    return sample_groups_[idx];
  }

private:
  std::vector<typename PointSampleGroup<FloatT>::Ptr> sample_groups_;
};

/**
 * @class descartes_planner::BDSPGraphPlanner
 * @brief Planner implementation that uses the dijkstra shortest path algorithm from the boost library *
 * @tparam FloatT Use float or double
 *
 */
template <typename FloatT>
class BDSPGraphPlanner
{
public:

  /**
   * @brief Creates the Graph planner
   * @param container       The container implementation to use, pass nullptr to use default.
   * @param report_failures Whether or not to report failed points and edges.  Use getFailedPoints and getFailedEdges in
   *                        order to get the indices where the planner failed.
   */
  BDSPGraphPlanner(typename std::shared_ptr< SamplesContainer<FloatT> > container = nullptr, bool report_failures = false);

  virtual ~BDSPGraphPlanner();

  /**
   * @brief Builds the graph
   * @param points          A vector of point samplers
   * @param edge_evaluator  A speed evaluator
   * @return True on success false otherwise
   */
  bool build(std::vector< typename PointSampler<FloatT>::Ptr >& points,
             typename EdgeEvaluator<FloatT>::ConstPtr edge_evaluator);

  /**
   * @brief Builds the graph
   * @param points          A vector of point samplers
   * @param edge_evaluator  A vector of speed evaluators, the vector size should be one less than the point samplers vector
   * @return True on success false otherwise
   */
  bool build(std::vector< typename PointSampler<FloatT>::Ptr >& points,
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

  typename EdgeEvaluator<FloatT>::ConstPtr getEdgeEvaluator(std::uint32_t idx);

  std::vector< EdgeProperties<FloatT> > filterDisconnectedEdges(const std::vector< EdgeProperties<FloatT> >& edges,
                                                                const std::map<int, VertexProperties>& connected_src_vertices,
                                                                std::uint32_t current_vertex_count) const;

  void setup(std::vector< typename PointSampler<FloatT>::Ptr >& points,
             std::vector<typename EdgeEvaluator<FloatT>::ConstPtr>& edge_evaluators);


  typedef typename boost::adjacency_list<boost::vecS,         /** @brief  edge container */
                                boost::vecS,                  /** @brief vertex_container */
                                boost::directedS,             /** @brief allows and out_edge only */
                                VertexProperties,     /** @brief vertex structure */
                                EdgeProperties<FloatT>        /** @brief edge structure */
                                > GraphT;

  void writeGraphLogs(const std::vector<FloatT>& weights,
                    const std::vector<typename GraphT::vertex_descriptor>& predecessors);

  GraphT graph_;
  std::vector< typename PointSampler<FloatT>::Ptr > points_;
  std::vector< typename EdgeEvaluator<FloatT>::ConstPtr > edge_evaluators_;
  std::map<int, VertexProperties> end_vertices_;
  typename std::shared_ptr< SamplesContainer<FloatT> > container_;

  bool report_failures_;
  std::vector<std::size_t> failed_points_;
  std::vector<std::size_t> failed_edges_;



};

} /* namespace descartes_planner */


#endif /* INCLUDE_DESCARTES_PLANNER_GRAPH_SOLVER_H_ */
