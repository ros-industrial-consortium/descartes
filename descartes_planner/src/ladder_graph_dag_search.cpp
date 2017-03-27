#include "descartes_planner/ladder_graph_dag_search.h"

namespace descartes_planner
{

DAGSearch::DAGSearch(const LadderGraph &graph)
  : graph_(graph)
{
  // On creating an object, let's allocate everything we need
  solution_.resize(graph.size());

  for (size_t i = 0; i < graph.size(); ++i)
  {
    const auto n_vertices = graph.rungSize(i);
    solution_[i].distance.resize(n_vertices);
    solution_[i].predecessor.resize(n_vertices);
  }
}

double DAGSearch::run(const std::vector<double>& seed_weights)
{

  if (!seed_weights.empty())
  {
    if (seed_weights.size() != solution_.front().distance.size())
      throw std::invalid_argument("Seed weights must match the size of initial row of joint solutions");

    solution_.front().distance = seed_weights;
  }
  else
  {
    // Cost to the first rung should be set to zero
    std::fill(solution_.front().distance.begin(), solution_.front().distance.end(), 0.0);
  }

  // Other rows initialize to zero
  for (size_type i = 1; i < solution_.size(); ++i)
  {
    std::fill(solution_[i].distance.begin(), solution_[i].distance.end(), std::numeric_limits<double>::max());
  }

  // Now we iterate over the graph in 'topological' order
  for (size_type rung = 0; rung < solution_.size() - 1; ++rung)
  {
    const auto n_vertices = graph_.rungSize(rung);
    const auto next_rung = rung + 1;
    // For each vertex in the out edge list
    for (size_t index = 0; index < n_vertices; ++index)
    {
      const auto u_cost = distance(rung, index);
      const auto& edges = graph_.getEdges(rung)[index];
      // for each out edge
      for (const auto& edge : edges)
      {
        auto dv = u_cost + edge.cost; // new cost
        if (dv < distance(next_rung, edge.idx))
        {
          distance(next_rung, edge.idx) = dv;
          predecessor(next_rung, edge.idx) = index; // the predecessor's rung is implied to be the current rung
        }
      }
    } // vertex for loop
  } // rung for loop

  return *std::min_element(solution_.back().distance.begin(), solution_.back().distance.end());
}

std::vector<DAGSearch::predecessor_t> DAGSearch::shortestPath() const
{
  auto min_it = std::min_element(solution_.back().distance.begin(), solution_.back().distance.end());
  auto min_idx = std::distance(solution_.back().distance.begin(), min_it);
  assert(min_idx >= 0);

  std::vector<predecessor_t> path (solution_.size());

  size_type current_rung = path.size() - 1;
  size_type current_index = min_idx;

  for (unsigned i = 0; i < path.size(); ++i)
  {
    auto count = path.size() - 1 - i;
    assert(current_rung == count);
    path[count] = current_index;
    current_index = predecessor(current_rung, current_index);
    current_rung -= 1;
  }

  return path;
}

} // namespace descartes_planner
