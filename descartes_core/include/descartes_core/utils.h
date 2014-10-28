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

#ifndef UTILS_H_
#define UTILS_H_

#include <boost/shared_ptr.hpp>

/** \def DESCARTES_CLASS_FORWARD
    Macro that forward declares a class XXX, and also defines two shared ptrs with named XXXPtr and XXXConstPtr  */

#define DESCARTES_CLASS_FORWARD(C)                \
    class C;                        \
    typedef boost::shared_ptr<C> C##Ptr;        \
    typedef boost::shared_ptr<const C> C##ConstPtr;    \

#endif /* UTILS_H_ */
