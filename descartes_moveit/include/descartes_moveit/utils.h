/*
 * utils.h
 *
 *  Created on: Feb 1, 2019
 *      Author: Jorge Nicho
 */
/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2019, Jorge Nicho
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

#ifndef INCLUDE_DESCARTES_MOVEIT_UTILS_H_
#define INCLUDE_DESCARTES_MOVEIT_UTILS_H_

#include <Eigen/Core>

namespace descartes_moveit
{

/**
 * @brief utility function that converts an Eigen Affine Transform type to an Isometry one.
 * @param t The Affine transform
 * @return  An Isometry transform
 */
template <class T>
static Eigen::Transform<T,3,Eigen::Isometry> toIsometry(const Eigen::Transform<T,3,Eigen::Affine>& t)
{
  Eigen::Transform<T,3,Eigen::Isometry> o;
  o.translation() = t.translation();
  o.linear() = t.rotation();
  return std::move(o);
}

/**
 * @brief workaround to allow compatibility with previous versions of MoveIt! that used Affine3 as the
 *  preferred transform type.
 * @param t The Affine transform
 * @return  An Isometry transform
 */
template <class T>
static Eigen::Transform<T,3,Eigen::Isometry> toIsometry(const Eigen::Transform<T,3,Eigen::Isometry>& t)
{
  return std::move(Eigen::Transform<T,3,Eigen::Isometry>(t));
}

}



#endif /* INCLUDE_DESCARTES_MOVEIT_UTILS_H_ */
