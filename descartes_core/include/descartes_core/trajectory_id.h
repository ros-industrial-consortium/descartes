#ifndef TRAJECTORY_ID_H
#define TRAJECTORY_ID_H

#include <boost/atomic.hpp>
#include <iostream>

namespace descartes_core
{

template<typename T>
struct IdGenerator;

template<>
struct IdGenerator<uint64_t>
{
  typedef uint64_t value_type;

  static value_type make_nil()
  {
    return 0;
  }

  static value_type make_id()
  {
    return counter_++;
  }

  static bool is_nil(value_type id)
  {
    return id == 0;
  }

private:
  static boost::atomic<value_type> counter_;
};

template<typename T>
class TrajectoryID_
{
public:
  typedef T value_type;

  TrajectoryID_(value_type id)
    : id_(id)
  {}

  TrajectoryID_()
  {}

  bool is_nil() const { return IdGenerator<value_type>::is_nil(id_); }

  value_type value() const { return id_; }

  static TrajectoryID_<value_type> make_id()
  {
    return TrajectoryID_<value_type>( IdGenerator<value_type>::make_id() );
  }

  static TrajectoryID_<value_type> make_nil()
  {
    return TrajectoryID_<value_type>( IdGenerator<value_type>::make_nil() );
  }

private:
  value_type id_;  
};

//////////////////////
// Helper Functions //
//////////////////////

template<typename T>
inline bool operator==(TrajectoryID_<T> lhs, TrajectoryID_<T> rhs)
{
  return lhs.value() == rhs.value();
}

template<typename T>
inline bool operator!=(TrajectoryID_<T> lhs, TrajectoryID_<T> rhs)
{
  return !(lhs == rhs);
}

template<typename T>
inline bool operator<(TrajectoryID_<T> lhs, TrajectoryID_<T> rhs)
{
  return lhs.value() < rhs.value();
}

template<typename T>
inline std::ostream& operator<<(std::ostream& os, TrajectoryID_<T> id)
{
  os << "ID" << id.value();
  return os;
}

typedef TrajectoryID_<uint64_t> TrajectoryID;

} // end namespace descartes_core


#endif
