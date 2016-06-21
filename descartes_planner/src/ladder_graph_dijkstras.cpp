#include "descartes_planner/ladder_graph_dijkstras.h"
#include <boost/heap/d_ary_heap.hpp>

namespace descartes_planner
{

struct ValueKey
{
  ValueKey()=default;
  ValueKey(VD v, double c) noexcept
    : vertex(v), cost(c) {}

  VD vertex;
  double cost;

  inline bool operator<(const ValueKey& rhs) const noexcept
  {
    return cost > rhs.cost;
  }
};

using BinaryHeap = boost::heap::d_ary_heap<ValueKey, boost::heap::arity<2>, boost::heap::mutable_<true> >;

DijkstrasSearch::DijkstrasSearch(const LadderGraph &graph)
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
    solution_[i].colors.resize(n_vertices);
    n += n_vertices;
  }

  N = n;
}

double DijkstrasSearch::run()
{
  using HeapType = BinaryHeap;
  using handle_t = typename HeapType::handle_type;

  // init
  for (auto& rung : solution_)
  {
    std::fill(rung.distance.begin(), rung.distance.end(), std::numeric_limits<double>::infinity());
    std::fill(rung.colors.begin(), rung.colors.end(), WHITE);
  }

  std::vector<handle_t> handles (N);
  HeapType heap; // In my initial testing, found reserve made little to no difference

  // Insert all of the first row's points
  for (auto i = 0u; i < solution_.front().distance.size(); ++i)
  {
    const auto src = VD{0u, i}; // {rung, index}
    handles[index(src)] = heap.push( ValueKey{src, 0.0} );
    distance(src) = 0.0;
  }

  while (!heap.empty())
  {
    const auto p = heap.top();
    heap.pop();

    const auto u = p.vertex; // VD
    const auto u_cost = p.cost;

    // now we find edges from u
    const auto& edges = graph_.getEdges(u.rung)[u.index];

    for (const auto& edge : edges)
    {
      auto v = VD{u.rung + 1, edge.idx};
      double dv = edge.cost + u_cost; // new cost to this point
      if (dv < distance(v)) // if the new cost is lower, we replace it
      {
        distance(v) = dv;
        predecessor(v) = u;

        if (color(v) == WHITE)
        {
          color(v) = GRAY;
          handles[index(v)] = heap.push( ValueKey(v, distance(v)) );
        }
        else
        {
          heap.increase(handles[index(v)], ValueKey(v, distance(v)) );
        }
      } // if-cost-lower block
    } // edge explore loop
    color(u) = BLACK;
  } // main loop

  return *std::min_element(solution_.back().distance.begin(), solution_.back().distance.end());
}

std::vector<unsigned> DijkstrasSearch::shortestPath() const
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
