/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2018, Southwest Research Institute
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

#ifndef DESCARTES_TEST_TRAJECTORY_MAKER_H
#define DESCARTES_TEST_TRAJECTORY_MAKER_H

#include "descartes_trajectory/cart_trajectory_pt.h"
#include "descartes_core/utils.h"

namespace descartes_tests
{
// Makes a linear trajectory with a given tool speed
std::vector<descartes_core::TrajectoryPtPtr> makeConstantVelocityTrajectory(const Eigen::Vector3d& start,
                                                                            const Eigen::Vector3d& stop,
                                                                            double tool_vel, size_t n_steps);

// Make a pattern that moves in a line but that oscillates along the
// y axis as it moves
std::vector<descartes_core::TrajectoryPtPtr> makeZigZagTrajectory(double x_start, double x_stop, double y_amplitude,
                                                                  double tool_vel, size_t n_steps);
}

#endif
