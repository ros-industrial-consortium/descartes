#include "descartes_planner/ladder_graph_dag_search.h"

namespace descartes_planner
{

DAGSearch::DAGSearch(const LadderGraph &graph)
  : graph_(graph)
{
  // On creating an object, let's allocate everything we need
  solution_.resize(graph.size());

  size_t n = 0; // rolling count

  for (size_t i = 0; i < graph.size(); ++i)
  {
    const auto n_vertices = graph.getRung(i).data.size() / graph.dof();
    solution_[i].n_start = n;
    solution_[i].distance.resize(n_vertices);
    solution_[i].predecessor.resize(n_vertices);
    n += n_vertices;
  }

  N = n;
}

double DAGSearch::run()
{
  // Cost to the first rung should be set to zero
  std::fill(solution_.front().distance.begin(), solution_.front().distance.end(), 0.0);
  // Other rows initialize to zero
  for (size_t i = 1; i < solution_.size(); ++i)
  {
    std::fill(solution_[i].distance.begin(), solution_[i].distance.begin(), std::numeric_limits<double>::max());
  }

  // Now we iterate over the graph in 'topological' order
  for (size_t rung = 0; rung < solution_.size() - 1; ++rung)
  {
    const auto n_vertices = graph_.rungSize(rung);
    // For each vertex in the out edge list
    for (size_t index = 0; index < n_vertices; ++index)
    {
      const auto u = VD{rung, index};
      const auto u_cost = distance(u);
      const auto& edges = graph_.getEdges(rung)[index];
      // for each out edge
      for (const auto& edge : edges)
      {
        auto v = VD{rung + 1, edge.idx};
        auto dv = u_cost + edge.cost; // new cost
        if (dv < distance(v))
        {
          distance(v) = dv;
          predecessor(v) = u;
        }
      }
    } // vertex for loop
  } // rung for loop

  return *std::min_element(solution_.back().distance.begin(), solution_.back().distance.end());
}

std::vector<unsigned> DAGSearch::shortestPath() const
{
  auto min_it = std::min_element(solution_.back().distance.begin(), solution_.back().distance.end());
  auto min_idx = std::distance(solution_.back().distance.begin(), min_it);
  assert(min_idx >= 0);

  std::vector<unsigned> path (solution_.size());

  VD vd {static_cast<unsigned>(path.size() - 1), static_cast<unsigned>(min_idx)};
  for (unsigned i = 0; i < path.size(); ++i)
  {
    auto count = path.size() - 1 - i;
    assert(vd.rung == count);
    path[count] = vd.index;
    vd = predecessor(vd);
  }

  return path;
}

} // namespace descartes_planner
