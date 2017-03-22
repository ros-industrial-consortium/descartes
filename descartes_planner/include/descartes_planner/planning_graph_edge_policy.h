#ifndef PLANNING_GRAPH_EDGE_POLICY_H
#define PLANNING_GRAPH_EDGE_POLICY_H

#include "descartes_planner/ladder_graph.h"

namespace descartes_planner
{

struct DefaultEdgesWithTime
{
 DefaultEdgesWithTime(const size_t n_start,
                      const size_t n_end,
                      const size_t dof,
                      const double upper_tm,
                      const std::vector<double>& joint_vel_limits)
    : results_(n_start)
    , edge_scratch_(n_end)
    , max_dtheta_(dof)
    , delta_buffer_(dof)
    , dof_(dof)
    , count_(0)
    , has_edges_(false)
  {
   std::transform(joint_vel_limits.cbegin(), joint_vel_limits.cend(), max_dtheta_.begin(), [upper_tm] (double v) {
                    return std::min(1.0, v * upper_tm);
                  });
  }

  inline void consider(const double* const start, const double* const stop, size_t index) noexcept
  {
    for (size_t i = 0; i < dof_; ++i)
    {
      delta_buffer_[i] = std::abs(start[i] - stop[i]);
      if (delta_buffer_[i] > max_dtheta_[i]) return;
    }

    auto cost = std::accumulate(delta_buffer_.cbegin(), delta_buffer_.cend(), 0.0);
    edge_scratch_[count_].cost = cost;
    edge_scratch_[count_].idx = static_cast<unsigned>(index);
    count_++;
  }

  inline void next(const size_t i)
  {
    results_[i].assign(edge_scratch_.cbegin(), edge_scratch_.cbegin() + count_);
    has_edges_ = has_edges_ || count_ > 0;
    count_ = 0;
  }

  inline std::vector<LadderGraph::EdgeList>& result() noexcept { return results_; }

  inline bool hasEdges() const noexcept { return has_edges_; }

  std::vector<LadderGraph::EdgeList> results_;
  LadderGraph::EdgeList edge_scratch_; // pre-allocated space to work in
  std::vector<double> max_dtheta_;
  std::vector<double> delta_buffer_;
  size_t dof_;
  unsigned count_;
  bool has_edges_;
};

struct CustomEdgesWithTime : public DefaultEdgesWithTime
{
  CustomEdgesWithTime(const size_t n_start,
                      const size_t n_end,
                      const size_t dof,
                      const double upper_tm,
                      const std::vector<double>& joint_vel_limits,
                      descartes_planner::CostFunction fn)
    : DefaultEdgesWithTime(n_start, n_end, dof, upper_tm, joint_vel_limits)
    , custom_cost_fn(fn)
  {}

  inline void consider(const double * const start, const double * const stop, const size_t index) noexcept
  {
    for (size_t i = 0; i < dof_; ++i)
    {
      delta_buffer_[i] = std::abs(start[i] - stop[i]);
      if (delta_buffer_[i] > max_dtheta_[i]) return;
    }

    double cost = custom_cost_fn(start, stop);
    edge_scratch_[count_++] = {cost, static_cast<unsigned>(index)};
  }

  descartes_planner::CostFunction custom_cost_fn; // TODO: Header doesn't stand on its own
};

struct DefaultEdgesWithoutTime
{
  DefaultEdgesWithoutTime(const size_t n_start,
                       const size_t n_end,
                       const size_t dof)
     : results_(n_start)
     , dof_(dof)
     , count_(0)
     , layer_(0)
  {
    for (auto& edges : results_)
    {
      edges.resize(n_end);
    }
  }

  inline bool hasEdges() const { return true; }

  // no-op
  inline void next(const size_t) { layer_++; count_ = 0; }

  inline std::vector<LadderGraph::EdgeList>& result() noexcept { return results_; }

  inline void consider(const double* const start, const double* const stop, const size_t index) noexcept
  {
    double cost = 0.0;
    for (size_t i = 0; i < dof_; ++i)
      cost += std::abs(start[i] - stop[i]);

    results_[layer_][count_].cost = cost;
    results_[layer_][count_].idx = static_cast<unsigned>(index);
    count_++;
  }

  std::vector<LadderGraph::EdgeList> results_;
  size_t dof_;
  size_t count_;
  size_t layer_;
};

struct CustomEdgesWithoutTime : public DefaultEdgesWithoutTime
{
  CustomEdgesWithoutTime(const size_t n_start,
                       const size_t n_end,
                       const size_t dof,
                          descartes_planner::CostFunction fn)
    : DefaultEdgesWithoutTime(n_start, n_end, dof), custom_cost_fn(fn)
  {}

  inline void consider(const double* const start, const double* const stop, const size_t index) noexcept
  {
    results_[layer_][count_].cost = custom_cost_fn(start, stop);
    results_[layer_][count_].idx = static_cast<unsigned>(index);
    count_++;
  }

  descartes_planner::CostFunction custom_cost_fn; // TODO: Header doesn't stand on its own
};

}

#endif // PLANNING_GRAPH_EDGE_POLICY_H
