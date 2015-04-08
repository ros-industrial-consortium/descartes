/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2015, Southwest Research Institute
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
 *
 *
 * This file defines methods that can be used to generate seed values for iterative
 * numerical solvers used by Moveit for a generic robot.
 *
 * Users can call the seed::findSeedStates() function with a robot state object, a
 * particular Moveit move group, and a series of joint pairs. These joint pairs
 * should define arm configurations such as elbow up or elbow down. For many 6 DOF
 * robots, joints 2 & 3 and joints 4 & 6 (starting counting at 1) will form elbow
 * and wrist configurations.
 *
 */

#include <descartes_moveit/seed_search.h>

#include <ros/ros.h>

using namespace descartes_moveit;

typedef std::vector<unsigned> BijectionVec;

/**
 * @brief A ComboHandle is a (relatively) light weight and precise way
 *        to represent a joint rotation permutation. Can work with up to
 *        255 joint discretizations and a lot of joints.
 */
struct ComboHandle
{
  ComboHandle(size_t n)
    : combos_(n)
  {}

  uint8_t& operator[](size_t idx)
  {
    return combos_[idx];
  }

  const uint8_t& operator[](size_t idx) const
  {
    return combos_[idx];
  }

  const std::size_t size() const { return combos_.size(); }

private:
  std::vector<uint8_t> combos_;
};

/**
 * @brief The CombinationManager class is responsible for creating
 *        joint permutations given the number of joints and the
 *        number of discretizations per joint.
 */
class CombinationManager
{
public:
  CombinationManager(unsigned n_joints, unsigned n_discs)
    : n_joints_(n_joints)
    , n_discs_(n_discs)
  {}

  size_t count() const
  {
    size_t result = 1;
    for (size_t i = 0; i < n_joints_; ++i) result *= n_discs_;
    return result;
  }

  ComboHandle fromNumber(size_t n)
  {
    ComboHandle result (n_joints_);
    for (size_t i = 0; i < n_joints_; i++)
    {
      result[n_joints_-1-i] = static_cast<uint8_t>(n % n_discs_);
      n /= n_discs_;
    }
    return result;
  }

private:
  unsigned n_joints_;
  unsigned n_discs_;
};

/**
 * @brief Comparison operator so we can use ComboHandles in std::set
 */
bool operator<(const ComboHandle& lhs, const ComboHandle& rhs)
{
  for (std::size_t i = 0; i < lhs.size(); ++i)
  {
    if ( lhs[i] != rhs[i] )
    {
      return lhs[i] < rhs[i];
    }
  }
  // equal
  return false;
}

// Simple container for holding info necessary for converting from handles to joint configs
class ConfigCreator
{
public:
  /**
   * @brief ConfigCreator constructor
   * @param initial_joint_pos The values for all joints that are not being permuted
   * @param bij The bijection vector where the first values in the bijection are the
   *            joints being permuted
   * @param min_angle The lower limit of the joint angles
   * @param step_angle Joint angle discretization
   */
  ConfigCreator(const std::vector<double> initial_joint_pos,
                const BijectionVec& bij, double min_angle, double step_angle)
    : initial_joint_positions_(initial_joint_pos)
    , bij_(bij)
    , min_angle_(min_angle)
    , step_angle_(step_angle)
  {}

  seed::JointConfig toJointConfig(const ComboHandle& handle) const
  {
    seed::JointConfig result;
    result.resize(bij_.size());

    std::size_t i;
    for (i = 0; i < handle.size(); i++)
    {
      result[bij_[i]] = min_angle_ + handle[i] * step_angle_;
    }

    for (; i < bij_.size(); ++i)
    {
      result[bij_[i]] = initial_joint_positions_[bij_[i]];
    }

    return result;
  }

private:
  std::vector<double> initial_joint_positions_;
  BijectionVec bij_;
  double min_angle_;
  double step_angle_;
};

/**
 * @brief Forward kinematics helper
 */
bool doFK(moveit::core::RobotState& state,
          const moveit::core::JointModelGroup* group,
          const std::string& tool,
          const seed::JointConfig& joint_pose,
          Eigen::Affine3d& result)
{
  state.setJointGroupPositions(group, joint_pose);
  if (!state.knowsFrameTransform(tool))
  {
    ROS_WARN("No transform to this tool frame");
    return false;
  }

  if (!state.satisfiesBounds())
  {
    ROS_WARN("Joint angles do not satisfy robot bounds");
    return false;
  }

  result = state.getFrameTransform(tool);
  return true;
}

/**
 * @brief Inverse kinematics helper. Returns the solution in the result parameter.
 *        May return false if IK failed.
 */
bool doIK(moveit::core::RobotState& state,
          const moveit::core::JointModelGroup* group,
          const std::string& group_name,
          const std::string& tool,
          const Eigen::Affine3d& pose,
          const seed::JointConfig& seed,
          seed::JointConfig& result)
{
  state.setJointGroupPositions(group_name, seed);
  if (!state.setFromIK(group, pose, tool, 1, 0.01))
  {
    return false;
  }
  state.copyJointGroupPositions(group, result);
  return true;
}

/**
 * @brief Returns true if the determinant of the jacobian is not zero.
 *        Returns false if it is close to zero (and robot is near singularity)
 */
bool checkSingularity(moveit::core::RobotState& state,
                      const moveit::core::JointModelGroup* group)
{
 Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
 Eigen::MatrixXd jacobian;
 state.getJacobian(group, state.getLinkModel(group->getLinkModelNames().back()),
                   reference_point_position, jacobian);

 return std::abs(jacobian.determinant()) > 0.0001;
}

// Compares the first n_compare bijection joint values from a and b and tests
// whether they are within 45 degrees of one another
inline bool sameJointConfig(const seed::JointConfig& a, const seed::JointConfig& b,
                            const BijectionVec& bij, unsigned n_compare)
{
  for (std::size_t i = 0; i < n_compare; i++)
  {
    if (std::fmod(std::abs(a[bij[i]] - b[bij[i]]), 2*M_PI) > M_PI_4) return false;
  }
  return true;
}

// Tests whether a given joint configuration, c, is similar to any joint set already
// in set using the given bijection values and number of joints.
inline bool inJointSet(const seed::JointConfig& c, const seed::JointConfigVec& set,
                       const BijectionVec& bij, unsigned n_compare)
{
  for (std::size_t i = 0; i < set.size(); ++i)
  {
    if (sameJointConfig(c, set[i], bij, n_compare)) return true;
  }
  return false;
}

// Creates a bijection vector which puts the permuted pair of joints
// up front.
BijectionVec createBijection(const seed::JointPair& pair, unsigned dof)
{
  BijectionVec bij;
  bij.reserve(dof);

  // add first two joints
  bij.push_back(pair.first);
  bij.push_back(pair.second);

  for (std::size_t i = 0; i < dof; i++)
  {
    if (i != pair.first && i != pair.second)
      bij.push_back(i);
  }

  return bij;
}

// Uses ROS's random number generator to fill out a vector of doubles (length dof)
// with uniform values between min and max
std::vector<double> randomInitialState(unsigned dof, double min, double max)
{
  std::vector<double> result;
  result.reserve(dof);

  random_numbers::RandomNumberGenerator gen;

  for (std::size_t i = 0; i < dof; ++i)
  {
    result.push_back(gen.uniformReal(min, max));
  }
  return result;
}

/**
 * @brief Returns possible seed states generated from permuting the given pair of joints
 *        (using joint numbering starting at 0)
 */
seed::JointConfigVec findSeedStatesForPair(moveit::core::RobotStatePtr state,
                                           const std::string& group_name,
                                           const std::string& tool_frame,
                                           const seed::JointPair& pair)
{
  const moveit::core::JointModelGroup* group = state->getJointModelGroup(group_name);

  // Hard-coded constants for joint iteration
  // Should be replaced by interface that uses moveit's joint tolerances
  // on a per joint basis
  const double min_angle = -M_PI;
  const double step_angle = M_PI_2;
  const unsigned steps = 4;
  const unsigned n_permuted_joints = 2;

  const unsigned dof = group->getActiveJointModelNames().size();

  // Create bijection and initial state (for non-permuted joints)
  BijectionVec bij = createBijection(pair, dof);
  std::vector<double> init_state = randomInitialState(dof, -M_PI_2, M_PI_2);

  // Create classes that will create permutations and
  // the resulting joint configurations
  CombinationManager combos (n_permuted_joints, steps);
  ConfigCreator configs (init_state, bij, min_angle, step_angle);

  // A set of joint permutation handles that lead to unique IK solutions for one
  // of the tested poses. This set, when transformed to joint configurations, is
  // the result of this function
  std::set<ComboHandle> global_seed_states;

  // Outer loop -> for every permutation calculate pose
  for (std::size_t i = 0; i < combos.count(); ++i)
  {
    auto round_handle = combos.fromNumber(i);
    auto round_config = configs.toJointConfig(round_handle);

    // Data structures for this round alone
    std::vector<ComboHandle> this_round_seeds;
    seed::JointConfigVec this_round_iks;

    // Forward kinematics
    Eigen::Affine3d target_pose;
    if (!doFK(*state, group, tool_frame, round_config, target_pose))
    {
      ROS_DEBUG_STREAM("No FK for pose " << i);
      continue;
    }

    // Check to make sure we're not in a singularity
    if (!checkSingularity(*state, group))
    {
      ROS_DEBUG_STREAM("Pose " << i << " at singularity.");
      continue;
    }

    // Add initial states to the iks/seeds for this point
    this_round_seeds.push_back(round_handle);
    this_round_iks.push_back(round_config);

    // Now try the seeds that have worked so far. The hope is to create
    // a minimal set of seeds that can solve for a given set of poses.
    for (const auto& s : global_seed_states)
    {
      // create joint config fom handle
      seed::JointConfig seed = configs.toJointConfig(s);
      seed::JointConfig ik;
      // perform ik
      if (!doIK(*state, group, group_name, tool_frame, target_pose, seed, ik))
      {
        continue;
      }
      // If we have a unique IK solution, then add to the iks seen this round
      if (!inJointSet(ik, this_round_iks, bij, n_permuted_joints))
      {
        this_round_iks.push_back(ik);
        this_round_seeds.push_back(s);
      }
    }

    // Now try the rest of the seeds
    for (std::size_t j = 0; j < combos.count(); j++)
    {
      // skip the situation where pose is generated from current seed
      // we already have that ik
      if (i == j) continue;

      auto handle = combos.fromNumber(j);
      // is the handle in the global solution set yet?
      auto count = global_seed_states.count(handle);
      if (count != 0) continue;

      seed::JointConfig seed = configs.toJointConfig(handle);
      seed::JointConfig ik;
      if (!doIK(*state, group, group_name, tool_frame, target_pose, seed, ik))
      {
        continue;
      }

      // If we have a new IK solution here, then it was generated
      // by a seed that has not yet been added to the overall seed
      // states to be returned
      if (!inJointSet(ik, this_round_iks, bij, n_permuted_joints))
      {
        this_round_iks.push_back(ik);
        this_round_seeds.push_back(handle);
        global_seed_states.insert(handle);
      }
    }

    ROS_DEBUG_STREAM("Calculated " << this_round_iks.size() << " unique IK states this round");
  } // outer loop end

  // Consolidate into answer and return
  seed::JointConfigVec result;
  result.reserve(global_seed_states.size());
  for (auto it = global_seed_states.begin(); it != global_seed_states.end(); ++it)
  {
    result.push_back(configs.toJointConfig(*it));
  }
  return result;
}

/**
 * @brief Returns a sequence of seed states for iterative inverse kinematic solvers to use
 *        when 'sampling' the solution space of a pose. These seeds are generated by
 *        iterating through all possible joint permutations of each pair of joints passed
 *        in by the user.
 * @param state Shared pointer to robot state used to perform FK/IK
 * @param group_name Name of the move group for which to generate seeds
 * @param tool_frame The name of the tool link in which to work with FK/IK
 * @param pairs A sequence of joint pairs used to generate the seed states.
 * @return A vector of seed states
 */
seed::JointConfigVec seed::findSeedStates(moveit::core::RobotStatePtr state,
                                          const std::string& group_name,
                                          const std::string& tool_frame,
                                          const JointPairVec& pairs)
{
  seed::JointConfigVec result;
  for (const auto& pair : pairs)
  {
    seed::JointConfigVec partial_answer = findSeedStatesForPair(state, group_name, tool_frame, pair);
    result.insert(result.end(), partial_answer.begin(), partial_answer.end());
  }
  return result;
}

seed::JointConfigVec seed::findRandomSeeds(moveit::core::RobotStatePtr state,
                                           const std::string& group_name,
                                           unsigned n)
{
  auto group = state->getJointModelGroup(group_name);

  JointConfigVec result;
  const unsigned max_iterations = n * 2;

  for (unsigned i = 0; i < max_iterations && result.size() < n; ++i)
  {
    JointConfig c;

    state->setToRandomPositions();
    state->copyJointGroupPositions(group, c);
    result.push_back(c);
  }
  return result;
}
