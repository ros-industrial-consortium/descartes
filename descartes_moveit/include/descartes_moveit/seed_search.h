#ifndef SEED_SEARCH_H
#define SEED_SEARCH_H

#include <moveit/robot_state/robot_state.h>

namespace descartes_moveit
{
namespace seed
{


typedef std::pair<unsigned, unsigned> JointPair;
typedef std::vector<JointPair> JointPairVec;
typedef std::vector<double> JointConfig;
typedef std::vector<JointConfig> JointConfigVec;


JointConfigVec findSeedStates(moveit::core::RobotStatePtr state,
                              const std::string& group_name,
                              const std::string& tool_frame,
                              const JointPairVec& pairs);

JointConfigVec findRandomSeeds(moveit::core::RobotStatePtr state,
                               const std::string& group_name,
                               unsigned n);

} // end namespace seed
} // end namespace descartes_moveit

#endif // SEED_SEARCH_H

