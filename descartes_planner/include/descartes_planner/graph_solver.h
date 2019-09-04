/*
 * graph_solver.h
 *
 *  Created on: Aug 30, 2019
 *      Author: jrgnicho
 */

#ifndef INCLUDE_DESCARTES_PLANNER_GRAPH_SOLVER_H_
#define INCLUDE_DESCARTES_PLANNER_GRAPH_SOLVER_H_

#include <boost/graph/adjacency_list.hpp>
#include "descartes_planner/common.h"
namespace descartes_planner
{

template <typename FloatT>
class GraphSolver
{
public:
  GraphSolver(typename EdgeEvaluator<FloatT>::ConstPtr edge_evaluator);
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


};

} /* namespace descartes_planner */


#endif /* INCLUDE_DESCARTES_PLANNER_GRAPH_SOLVER_H_ */
