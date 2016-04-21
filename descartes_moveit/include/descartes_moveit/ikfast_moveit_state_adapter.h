/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016, Jonathan Meyer
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

#ifndef IKFAST_MOVEIT_STATE_ADAPTER_H
#define IKFAST_MOVEIT_STATE_ADAPTER_H

#include "descartes_moveit/moveit_state_adapter.h"

namespace descartes_moveit
{

class IkFastMoveitStateAdapter : public descartes_moveit::MoveitStateAdapter
{
public:
  virtual ~IkFastMoveitStateAdapter() {}

  virtual bool getAllIK(const Eigen::Affine3d &pose, 
                        std::vector<std::vector<double> > &joint_poses) const;

};

} // end namespace 'descartes_moveit'
#endif
