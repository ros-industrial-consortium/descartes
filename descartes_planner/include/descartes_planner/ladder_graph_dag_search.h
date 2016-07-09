#ifndef DESCARTES_LADDER_GRAPH_DAG_SEARCH_H
#define DESCARTES_LADDER_GRAPH_DAG_SEARCH_H

#include "descartes_planner/ladder_graph.h"

namespace descartes_planner
{


// Directed Acyclic graph search
class DAGSearch
{
public:
  struct VD // Vertex Descriptor
  {
    unsigned rung;
    unsigned index;
  };
  explicit DAGSearch(const LadderGraph& graph);

  double run();

  std::vector<unsigned> shortestPath() const;

private:
  const LadderGraph& graph_;

  inline size_t index(VD a) const noexcept
  {
    return solution_[a.rung].n_start + a.index;
  }

  inline double& distance(VD v) noexcept
  {
    return solution_[v.rung].distance[v.index];
  }

  inline VD& predecessor(VD v) noexcept
  {
    return solution_[v.rung].predecessor[v.index];
  }

  inline const VD& predecessor(VD v) const noexcept
  {
    return solution_[v.rung].predecessor[v.index];
  }

  struct SolutionRung
  {
    size_t n_start;
    std::vector<double> distance;
    std::vector<VD> predecessor;
  };

  std::vector<SolutionRung> solution_;
  std::size_t N;
};
} // descartes_planner
#endif
