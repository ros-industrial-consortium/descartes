/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Dan Solomon
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
 * trajectory_pt_sampler.h
 *
 *  Created on: Oct 29, 2014
 *      Author: Dan Solomon
 */

#ifndef TRAJECTORY_PT_SAMPLER_H_
#define TRAJECTORY_PT_SAMPLER_H_

#include <descartes_core/trajectory_pt.h>
#include <descartes_core/utils.h>
#include <sensor_msgs/JointState.h>


namespace descartes_core
{

DESCARTES_CLASS_FORWARD(TrajectoryPtSampler);
DESCARTES_CLASS_FORWARD(TrajectoryPt);

class TrajectoryPtSampler
{
public:
  TrajectoryPtSampler() {};
  virtual ~TrajectoryPtSampler() {};

  /**@brief Initialize sampler data for @e pt
   * @param pt TrajectoryPt being initialized.
   */
  virtual
  bool init(const TrajectoryPt &pt) = 0;

  /**@brief Return joint solutions that satisfy a TrajectoryPt.
   *
   * Subsequent calls to this function return more solutions (if they remain. If sampler is complete, will return false).
   *
   * @param[out] solutions vector of JointStates.
   * @param[in] n Number of samples to perform (not necessarily equal to number of solutions returned). n=0 means get all remaining solutions.
   * @param[in] pt TrajectoryPt to sample from.
   * @return True if new solutions were successfully created.
   */
  virtual
  const void* sample(size_t n, const TrajectoryPt &pt) = 0;

};

} /* namespace descartes_core */

#endif /* TRAJECTORY_PT_SAMPLER_H_ */
