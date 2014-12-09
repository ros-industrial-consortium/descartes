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
 * joint_pt_sampler_base.h
 *
 *  Created on: Dec 2, 2014
 *      Author: Dan Solomon
 */

#ifndef JOINT_PT_SAMPLER_BASE_H_
#define JOINT_PT_SAMPLER_BASE_H_

#include <vector>
#include "descartes_core/utils.h"


namespace descartes_core
{

DESCARTES_CLASS_FORWARD(JointPtSamplerBase);

/**@brief JointPtSamplerBase class handles creating joint position samples.
 *
 * A sampler must A) have desired sample increment set and B) be initialized with nominal position, upper and lower bounds.
 * Then, @e sample() can be called repeatedly until all samples are returned.
 *
 */
class JointPtSamplerBase
{
public:
JointPtSamplerBase() {};
virtual ~JointPtSamplerBase() {};

/**@name Discretization
 * @{
 */
virtual
std::vector<double> getSampleIncrement() const = 0;
virtual
void setSampleIncrement(const std::vector<double> &values) = 0;
/** @} (end section) */

/**@brief Initialize sampler data
* @param[in] nominal
* @param[in] upper_bound
* @param[in] lower_bound
* @return True if init successful.
*/
virtual
bool initPositionData(const std::vector<double> &nominal, const std::vector<double> &upper_bound, const std::vector<double> &lower_bound) = 0;

/**@brief Return joint pt sample.
*
* Subsequent calls to this function return additional solutions (if they remain. If sampler is complete, will return false).
*
* @param[out] result Resultant joint state sample.
* @return True if new sample successfully generated.
*/
virtual
bool sample(std::vector<double> &result) = 0;

}; /* class JointPtSamplerBase */
}  /* namespace descartes_core */


#endif /* JOINT_PT_SAMPLER_BASE_H_ */
