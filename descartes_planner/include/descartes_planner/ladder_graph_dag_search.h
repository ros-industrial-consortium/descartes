#ifndef DESCARTES_LADDER_GRAPH_DAG_SEARCH_H
#define DESCARTES_LADDER_GRAPH_DAG_SEARCH_H

#include "descartes_planner/ladder_graph.h"

namespace descartes_planner
{

// Directed Acyclic graph search
class DAGSearch
{
public:
  using predecessor_t = unsigned;
  using size_type = std::size_t;

  explicit DAGSearch(const LadderGraph& graph);

  double run();

  std::vector<predecessor_t> shortestPath() const;

private:
  const LadderGraph& graph_;

  struct SolutionRung
  {
    std::vector<double> distance;
    std::vector<predecessor_t> predecessor;
  };

  inline double& distance(size_type rung, size_type index) noexcept
  {
    return solution_[rung].distance[index];
  }

  inline predecessor_t& predecessor(size_type rung, size_type index) noexcept
  {
    return solution_[rung].predecessor[index];
  }

  inline const predecessor_t& predecessor(size_type rung, size_type index) const noexcept
  {
    return solution_[rung].predecessor[index];
  }

  std::vector<SolutionRung> solution_;
};
} // descartes_planner
#endif
