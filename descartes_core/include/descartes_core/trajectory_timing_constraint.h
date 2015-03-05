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
/*
 * trajectory_pt_transition.h
 *
 *  Created on: Jun 5, 2014
 *      Author: Dan Solomon
 */

#ifndef TRAJECTORY_TIMING_CONSTRAINT_H
#define TRAJECTORY_TIMING_CONSTRAINT_H

namespace descartes_core
{
  /**
   * Timing constraints 
   */
  struct TimingConstraint
  {
    TimingConstraint()
      : lower_(0.0)
      , upper_(0.0)
    {}

    TimingConstraint(double lower, double upper)
      : lower_(lower)
      , upper_(upper)
    {}

    bool isSpecified() const { return upper_ > 0.0; }

    double lower_, upper_;
  };

} // end namespace descartes core

#endif /* TRAJECTORY_TIMING_CONSTRAINT_H */
