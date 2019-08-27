/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2018, Southwest Research Institute
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
#include <pluginlib/class_list_macros.h>
#include <descartes_planner/dense_planner.h>
#include <descartes_planner/sparse_planner.h>

PLUGINLIB_EXPORT_CLASS(descartes_planner::DensePlanner, descartes_core::PathPlannerBase)
PLUGINLIB_EXPORT_CLASS(descartes_planner::SparsePlanner, descartes_core::PathPlannerBase)
