/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016, Southwest Research Institute
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
 *  spline_interpolator.h
 *
 *  Created on: March 25, 2016
 *  Author: Jonathan Meyer
 */

#ifndef DESCARTES_SPLINE_INTERPOLATOR_H
#define DESCARTES_SPLINE_INTERPOLATOR_H

#include <vector>

namespace descartes_utilities
{

/**
 * Helper struct to store the parameters of a cubic spline:
 *   f(x) = a + bx + cx^2 + dx^3
 */
struct CubicSplineParams
{
  CubicSplineParams() = default;
  CubicSplineParams(double a, double b, double c, double d)
    : coeffs {a,b,c,d}
  {}
  
  double coeffs[4];

  double a() const { return coeffs[0]; }
  double b() const { return coeffs[1]; }
  double c() const { return coeffs[2]; }
  double d() const { return coeffs[3]; }

  /**
   * Computes f(x) at given x
   */
  double position(double x) const;

  /**
   * Computes df/dx at given x
   */
  double velocity(double x) const;

  /**
   * Computes ddf/ddx at given x
   */
  double acceleration(double x) const;

  /**
   * Computes the 'x' position that maximizes the velocity
   */
  double xMaxVelocity() const;

  /**
   * Computes the 'x' position that maximizes the acceleration
   */
  double xMaxAcceleration() const;
};

/**
 * Builds a sequence of cubic splines between each pair of sequential inputs
 * such that positions are obeyed absolutely, velocities are smooth & continous
 * and accelerations are continous.
 */
class SplineInterpolator
{
public:
  /**
   * The constructor builds the set of cubic splines for an input function defined by
   * 'x_data', and 'y_data'. 'x_data' and 'y_data' are assumed to be the same length (> 1).
   * 'x_data' must be sorted in ascending order.  
   */
  SplineInterpolator(const std::vector<double>& x_data, const std::vector<double>& y_data,
                     double init_velocity = 0.0, double final_velocity = 0.0);

  /**
   * Query the position of the curve at a given time or 'x' coordinate. Times outside the
   * bounds determined by the first/last point in 'x_data' are interpolated based on the
   * first/last cubic spline segment respectively.
   */
  double position(double x) const;
  
  /**
   * Query the velocity of the curve at a given time or 'x' coordinate. Times outside the
   * bounds determined by the first/last point in 'x_data' are interpolated based on the
   * first/last cubic spline segment respectively.
   */
  double velocity(double x) const;

  /**
   * Query the acceleration of the curve at a given time or 'x' coordinate. Times outside the
   * bounds determined by the first/last point in 'x_data' are interpolated based on the
   * first/last cubic spline segment respectively.
   */
  double acceleration(double x) const;

  /**
   * Returns the maximum velocity in the spline-interpolated trajectory.
   * @return A std::pair where the first element is the magnitude of the maximum velocity
   *         and the second value is the trajectory segment in which this velocity occurs.
   */
  std::pair<double, unsigned> maxVelocity() const;
  
  /**
   * Returns the maximum acceleration in the spline-interpolated trajectory.
   * @return A std::pair where the first element is the magnitude of the maximum acceleration
   *         and the second value is the trajectory segment in which this velocity occurs.
   */  
  std::pair<double, unsigned> maxAcceleration() const;
  
  /**
   * Returns a reference to the 'x_data' used to construct the cubic splines.
   */
  const std::vector<double> xData() const { return x_data_; }

  /**
   * Returns a reference to the cubic spline segments constructed from 'x_data' and 'y_data'. 
   */
  const std::vector<CubicSplineParams> splineParams() const { return spline_params_; }

private:
  /**
   * Returns the cubic segment in which 'x' resides. If x is greater than the
   * last x value in the input, the last segment is returned. If x is less than
   * the x value in the input, the first is returned.
   */
  unsigned search(double x) const;
  
  /**
   * Returns the maximum velocity for a given spline segment.
   */
  double maxSegmentVelocity(unsigned idx) const;
  
  /**
   * Returns the maximum acceleration for a given spline segment.
   */
  double maxSegmentAcceleration(unsigned idx) const;

  std::vector<double> x_data_;
  std::vector<CubicSplineParams> spline_params_;
};

}

#endif
