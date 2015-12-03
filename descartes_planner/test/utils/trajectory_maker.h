#ifndef DESCARTES_TEST_TRAJECTORY_MAKER_H
#define DESCARTES_TEST_TRAJECTORY_MAKER_H

#include "descartes_trajectory/cart_trajectory_pt.h"
#include "descartes_core/utils.h"

namespace descartes_tests
{

// Makes a linear trajectory with a given tool speed
std::vector<descartes_core::TrajectoryPtPtr>
makeConstantVelocityTrajectory(const Eigen::Vector3d& start,
                               const Eigen::Vector3d& stop,
                               double tool_vel,
                               size_t n_steps);
}

#endif
