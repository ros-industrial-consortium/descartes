/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef DESCARTES_LADDER_GRAPH_H
#define DESCARTES_LADDER_GRAPH_H

#include "descartes_core/trajectory_id.h"
#include "descartes_core/trajectory_timing_constraint.h"

namespace descartes_planner
{

struct __attribute__ ((__packed__)) Edge
{
  double cost; // transition cost from vertex who owns this object to 'idx' in next rung
  unsigned idx; // from THIS rung to 'idx' into the NEXT rung
};

struct Rung
{
  using EdgeList = std::vector<Edge>;

  descartes_core::TrajectoryID id; // corresponds to user's input ID
  descartes_core::TimingConstraint timing; // user input timing
  std::vector<double> data; // joint values stored in one contiguous array
  std::vector<EdgeList> edges;
};

/**
 * @brief LadderGraph is an adjacency list based, directed graph structure with vertices
 *        arranged into "rungs" which have connections only to vertices in the adjacent
 *        rungs. Assumes a fixed DOF.
 */
class LadderGraph
{
public:
  using size_type = std::size_t;
  using EdgeList = Rung::EdgeList;

  /**
   * @brief LadderGraph
   * @param dof The number of joints that constitute a single 'DOF'
   */
  explicit LadderGraph(size_type dof) noexcept
    : dof_(dof)
  {
    assert(dof != 0);
  }

  /**
   * @brief resize Resizes the internal ladder to have 'n_rung' rungs
   * @param n_rungs Number of individual rungs
   */
  void resize(size_type n_rungs)
  {
    rungs_.resize(n_rungs);
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
    assert(index < rungs_.size());
    return rungs_[index].edges;
  }

  size_type rungSize(size_type index) const noexcept
  {
    return getRung(index).data.size() / dof_;
  }

  /**
   * @brief numVertices Counts the total number of vertices in the graph
   */
  size_type numVertices() const noexcept
  {
    size_type count = 0; // Add the size of each rung d
    for (const auto& rung : rungs_) count += (rung.data.size() / dof_);
    return count;
  }

  /**
   * @brief indexOf returns a pair describing whether the given ID is in the graph and if so, what
   *        index it has.
   * @param id The ID to
   * @return std::pair(index, was_found)
   */
  std::pair<size_type, bool> indexOf(descartes_core::TrajectoryID id) const noexcept
  {
    auto it = std::find_if(rungs_.cbegin(), rungs_.cend(), [id] (const Rung& r) {
      return id == r.id;
    });
    if (it == rungs_.cend())
      return {0u, false};
    else
      return {static_cast<size_type>(std::distance(rungs_.cbegin(), it)), true};
  }

  /**
   * @brief isLast tests to see if a given index is the last one in the graph
   */
  bool isLast(size_type index) const noexcept
  {
    return index + 1 == size();
  }

  /**
   * @brief isFirst tests to see if given index is the first in the graph
   */
  bool isFirst(size_type index) const noexcept
  {
    return index == 0;
  }

  /**
   * @brief vertex returns a pointer to the data that constitutes the Nth vertex in the Jth row
   *        where N = index & J = rung
   */
  const double* vertex(size_type rung, size_type index) const
  {
    return getRung(rung).data.data() + (dof_ * index);
  }

  /**
   * @brief The number of rungs
   */
  size_type size() const noexcept
  {
    return rungs_.size();
  }

  size_type dof() const noexcept
  {
    return dof_;
  }

  /**
   * @brief assign Consumes the given edge list and assigns it to the rung-index given by 'rung'
   */
  void assignEdges(size_type rung, std::vector<EdgeList>&& edges) // noexcept?
  {
    getEdges(rung) = std::move(edges);
  }

  /**
   * @brief assignRung Special helper function to assign a solution set associated with a Descartes point &
   *        it's meta-info. Also resizes the associated edge list to the size of 'sols'.
   * @param sols All of the joint solutions for this point.
   */
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

  void removeRung(size_type index)
  {
    rungs_.erase(std::next(rungs_.begin(), index));
  }

  void clearVertices(size_type index)
  {
    rungs_[index].data.clear();
  }

  void clearEdges(size_type index)
  {
    rungs_[index].edges.clear();
  }

  /**
   * @brief insertRung Adds a new rung at the 'index'-th position. E.g., insertRung(0) will add a new
   *        rung to the beginning of the graph and the previous 0th index is now at 1.
   */
  void insertRung(size_type index)
  {
    rungs_.insert(std::next(rungs_.begin(), index), Rung() );
  }

  /**
   * @brief Clears all existing rungs & associated data
   */
  void clear()
  {
    rungs_.clear();
  }

private:
  const size_type dof_;
  std::vector<Rung> rungs_;
};
} // descartes_planner
#endif
