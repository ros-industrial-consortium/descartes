#ifndef DESCARTES_LADDER_GRAPH_H
#define DESCARTES_LADDER_GRAPH_H

#include "descartes_core/trajectory_id.h"
#include "descartes_core/trajectory_timing_constraint.h"

namespace descartes_planner
{

struct Rung
{
  descartes_core::TrajectoryID id; // corresponds to user's input ID
  descartes_core::TimingConstraint timing; // user input timing
  std::vector<double> data; // joint values stored in one contiguous array
};

struct __attribute__ ((__packed__)) Edge
{
  double cost;
  unsigned idx; // from THIS rung to 'idx' into the NEXT rung
};

/**
 * @brief The LadderGraph class
 */
class LadderGraph
{
public:
  using size_type = std::size_t;
  using EdgeList = std::vector<Edge>;

  explicit LadderGraph(size_type dof) noexcept
    : dof_(dof)
  {
    assert(dof != 0);
  }

  void allocate(size_type n_rungs)
  {
    rungs_.resize(n_rungs);
    edges_.resize(n_rungs);
  }

  Rung& getRung(size_type index) noexcept
  {
    return const_cast<Rung&>(static_cast<const LadderGraph&>(*this).getRung(index));
  }

  const Rung& getRung(size_type index) const noexcept
  {
    assert(index < rungs_.size());
    return rungs_[index];
  }

  std::vector<EdgeList>& getEdges(size_type index) noexcept // see p.23 Effective C++ (Scott Meyers)
  {
    return const_cast<std::vector<EdgeList>&>(static_cast<const LadderGraph&>(*this).getEdges(index));
  }

  const std::vector<EdgeList>& getEdges(size_type index) const noexcept
  {
    return edges_[index];
  }

  size_type rungSize(size_type index) const noexcept
  {
    return getRung(index).data.size() / dof_;
  }

  size_type numVertices() const noexcept
  {
    size_type count = 0; // Add the size of each rung d
    for (const auto& rung : rungs_) count += (rung.data.size() / dof_);
    return count;
  }

  std::pair<size_type, bool> indexOf(descartes_core::TrajectoryID id) const noexcept
  {
    auto it = std::find_if(rungs_.cbegin(), rungs_.cend(), [id] (const Rung& r) {
      return id == r.id;
    });
    if (it == rungs_.cend())
    {
      return {0u, false};
    }
    else
    {
      return {static_cast<size_type>(std::distance(rungs_.cbegin(), it)), true};
    }
  }

  bool isLast(size_type index) const noexcept
  {
    return index + 1 == size();
  }

  bool isFirst(size_type index) const noexcept
  {
    return index == 0;
  }

  const double* vertex(size_type rung, size_type index) const
  {
    return getRung(rung).data.data() + (dof_ * index);
  }

  size_type size() const noexcept
  {
    return rungs_.size();
  }

  size_type dof() const noexcept
  {
    return dof_;
  }

  // Mutate edges
  void assignEdgeList(size_type rung, size_type index, EdgeList&& out_edges) // noexcept?
  {
    getEdges(rung)[index] = std::move(out_edges);
  }

  void assignEdges(size_type rung, std::vector<EdgeList>&& edges) // noexcept?
  {
    getEdges(rung) = std::move(edges);
  }

  // Mutate Vertices
  void assignRung(size_type index, descartes_core::TrajectoryID id, descartes_core::TimingConstraint time,
                  const std::vector<std::vector<double>>& sols)
  {
    Rung& r = getRung(index);
    r.id = id;
    r.timing = time;
    r.data.reserve(sols.size() * dof_);
    for (const auto& sol : sols)
    {
      r.data.insert(r.data.end(), sol.cbegin(), sol.cend());
    }
    // Given this new vertex set, build an edge list for each
    getEdges(index).resize(r.data.size());
  }

  void removeVertices(size_type index)
  {
    rungs_.erase(std::next(rungs_.begin(), index));
  }

  void removeEdges(size_type index)
  {
    edges_.erase(std::next(edges_.begin(), index));
  }

  void removeRung(size_type index)
  {
    removeVertices(index);
    removeEdges(index);
  }

  void clearVertices(size_type index)
  {
    rungs_[index].data.clear();
  }

  void clearEdges(size_type index)
  {
    edges_[index].clear();
  }

  void insertRung(size_type index)
  {
    rungs_.insert(std::next(rungs_.begin(), index), Rung() );
    edges_.insert(std::next(edges_.begin(), index), std::vector<EdgeList>() );
  }

  void clear()
  {
    rungs_.clear();
    edges_.clear();
  }

private:
  const size_type dof_;
  std::vector<Rung> rungs_;
  std::vector<std::vector<EdgeList>> edges_;
};
} // descartes_planner
#endif
