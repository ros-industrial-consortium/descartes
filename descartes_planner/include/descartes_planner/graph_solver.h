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

template <typename FloatT>
class GraphSolver
{
public:
  GraphSolver(typename EdgeEvaluator<FloatT>::ConstPtr edge_evaluator,
              typename std::shared_ptr< SamplesContainer<FloatT> > container = nullptr);

  virtual ~GraphSolver();

  bool build(std::vector< typename PointSampler<FloatT>::Ptr >& points);
  bool solve(std::vector< typename PointSampleGroup<FloatT>::ConstPtr >& solution_points);

private:

  typedef typename boost::adjacency_list<boost::vecS,         /** @brief  edge container */
                                boost::vecS,                  /** @brief vertex_container */
                                boost::directedS,             /** @brief allows and out_edge only */
                                VertexProperties,     /** @brief vertex structure */
                                EdgeProperties<FloatT>        /** @brief edge structure */
                                > GraphT;

  GraphT graph_;
  std::vector< typename PointSampler<FloatT>::Ptr > points_;
  typename EdgeEvaluator<FloatT>::ConstPtr edge_evaluator_;
  std::map<std::size_t, VertexProperties> end_vertices_;
  typename std::shared_ptr< SamplesContainer<FloatT> > container_;


};

} /* namespace descartes_planner */


#endif /* INCLUDE_DESCARTES_PLANNER_GRAPH_SOLVER_H_ */
