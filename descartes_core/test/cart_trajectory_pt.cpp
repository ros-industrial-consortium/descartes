/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
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

#include "descartes_core/cart_trajectory_pt.h"
#include <gtest/gtest.h>


using namespace descartes_core;



TEST(CartTrajPt, construction)
{
CartTrajectoryPt def();
}

/*
TEST(CartTrajPt, getAllCartPoses)
{
  const double POS_TOL = 0.5;
  const double POS_INC = 0.1;

  const double ORIENT_TOL = 1.0;
  const double ORIENT_INC = 0.2;

  const double NUM_SAMPLED_POS = (POS_TOL/POS_INC) + 1;
  const double NUM_SAMPLED_ORIENT = (ORIENT_TOL/ORIENT_INC) + 1;


  CartTrajectoryPt fuzz_pos(TolerancedFrame( PositionTolerance(0, 0, 0, POS_TOL),
                                       OrientationTolerance(0, 0, 0, 0.0)),
                      POS_INC, ORIENT_INC);

  CartTrajectoryPt fuzz_orient(TolerancedFrame( PositionTolerance(0, 0, 0, 0.0),
                                       OrientationTolerance(0, 0, 0, ORIENT_TOL)),
                      POS_INC, ORIENT_INC);

  CartTrajectoryPt fuzz_both(TolerancedFrame( PositionTolerance(0, 0, 0, POS_TOL),
                                       OrientationTolerance(0, 0, 0, ORIENT_TOL)),
                      POS_INC, ORIENT_INC);

  EigenSTL::vector_Affine3d solutions;
}
*/
