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
 * cart_pt_sampler_base.h
 *
 *  Created on: Dec 2, 2014
 *      Author: Dan Solomon
 */

#ifndef CART_PT_SAMPLER_BASE_H_
#define CART_PT_SAMPLER_BASE_H_

#include <Eigen/Geometry>
#include <vector>
#include "descartes_core/utils.h"


namespace descartes_core
{

DESCARTES_CLASS_FORWARD(CartPtSamplerBase);

/**@brief CartPtSamplerBase class handles creating Cartesian frame samples.
 *
 * A sampler must be initialized with nominal position, upper and lower bounds, and desired discretization.
 * Then, @e sample() can be called repeatedly until all the samples are returned.
 *
 */
class CartPtSamplerBase
{
public:
CartPtSamplerBase() {};
virtual ~CartPtSamplerBase() {};

/**@name Discretization
 *
 * Sample increment describes how to increment the value of each sample.
 * For a Cartesian sampler, 6 sampling increment values are needed, 3 for position and 3 for rotation.
 * It is left up to the implementation as to how the rotation increments are used.
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
* @param[in] discretization
* @return True if init successful.
*/
virtual
bool initPositionData(const Eigen::Affine3d &nominal,
                      const Eigen::Vector3d &upper_position_bound, const Eigen::Vector3d &lower_position_bound,
                      const Eigen::Vector3d &upper_rpy_bound, const Eigen::Vector3d &lower_rpy_bound) = 0;

/**@brief Return Cartesian frame sample.
*
* Subsequent calls to this function return additional solutions (if they remain. If sampler is complete, will return false).
*
* @param[out] result Resultant joint state sample.
* @return True if any new samples were successfully generated.
*/
virtual
bool sample(Eigen::Affine3d &result) = 0;

}; /* class CartPtSamplerBase */
}  /* namespace descartes_core */


#endif /* CART_PT_SAMPLER_BASE_H_ */
