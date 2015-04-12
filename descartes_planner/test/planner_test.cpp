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
 
#include "descartes_planner/sparse_planner.h"
#include "descartes_planner/dense_planner.h"
#include "descartes_planner/planning_graph.h"

using namespace descartes_core;

using testing::Types;

namespace descartes_planner_test
{

descartes_planner::SparsePlanner SparsePlanner;
descartes_planner::DensePlanner DensePlanner;

template<class T>
class SparsePlannerTest : public descartes_planner_test::DescartesPlannerTest<T>{};

template<class T>
class DensePlannerTest : public descartes_planner_test::DescartesPlannerTest<T>{};

INSTANTIATE_TYPED_TEST_CASE_P(SparsePlannerTest, DescartesPlannerTest, SparsePlanner);
INSTANTIATE_TYPED_TEST_CASE_P(DensePlannerTest, DescartesPlannerTest, DensePlanner);

} //descartes_planner_test