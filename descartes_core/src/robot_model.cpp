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

#include "descartes_core/robot_model.h"

namespace descartes_core
{

bool RobotModel::equal(const std::vector<double> &lhs, const std::vector<double> &rhs,
                               const double tol)
{
  bool rtn = false;
  if( lhs.size() == rhs.size() )
  {
    rtn = true;
    for(size_t ii = 0; ii < lhs.size(); ++ii)
    {
      if(std::fabs(lhs[ii]-rhs[ii]) > tol)
      {
        rtn = false;
        break;
      }
    }

  }
  else
  {
    rtn = false;
  }
  return rtn;
}

} //descartes_core

