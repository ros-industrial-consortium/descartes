/**
  Copyright (c) 2016, Southwest Research Institute
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "descartes_utilities/spline_interpolator.h"
#include <cassert>
#include <cmath>
#include <algorithm>

/*
  This code does the heavy lifting of solving the constraints matrix.
  
  Taken partially from Daniel Stonier's ecl spline implementation:
    https://github.com/stonier/ecl_core/tree/indigo-devel/ecl_geometry

  which in turns draws the algorithm from:
    This algorithm can be found in page 115 of "Numerical Recipes in C"
    [The art of scientific computing] by Press, Vetterling, Tuekolsky, Flannery.
*/
static void fit_splines(const std::vector<double>& x_data, const std::vector<double>& y_data,
                        double init_velocity, double final_velocity, std::vector<double>& yddot_data)
{
  const auto n = x_data.size(); 
  std::vector<double> u(n);  // u is a temporary used in the algorithm

  // Initial boundary conditions
  yddot_data[0] = -0.5;
  u[0] = (3.0/(x_data[1]-x_data[0])) * ((y_data[1]-y_data[0])/(x_data[1]-x_data[0])-init_velocity);
  // Set up the tridiagonal matrix to solve for the accelerations
  for (unsigned int i = 1; i <= n-2; ++i) 
  {
      double sig = (x_data[i]-x_data[i-1]) / (x_data[i+1]-x_data[i-1]);
      double p = sig*yddot_data[i-1]+2.0;
      yddot_data[i] = (sig-1.0)/p;
      u[i] = (y_data[i+1]-y_data[i])/(x_data[i+1]-x_data[i]) -
      (y_data[i]-y_data[i-1])/(x_data[i]-x_data[i-1]);
      u[i] = (6.0*u[i]/(x_data[i+1]-x_data[i-1]) - sig*u[i-1])/p;
  }
  // Final boundary conditions
  double qn = 0.5;
  u[n-1] = (3.0/(x_data[n-1]-x_data[n-2])) * (final_velocity - (y_data[n-1]-y_data[n-2])/(x_data[n-1]-x_data[n-2]));

  // Back substitution loop of the tridiagonal algorithm
  yddot_data[n-1] = ( u[n-1] - qn*u[n-2]) / ( qn*yddot_data[n-2] + 1.0 );
  for ( int k = n-2; k >= 0; --k ) 
  {
    yddot_data[k] = yddot_data[k]*yddot_data[k+1] + u[k];
  }
}

////////////////////////////////
// Cubic Spline Segment Implemenation //
////////////////////////////////
double descartes_utilities::CubicSplineParams::position(double x) const
{
  return a() + b() * x + c() * std::pow(x, 2) + d() * std::pow(x, 3);
}

double descartes_utilities::CubicSplineParams::velocity(double x) const
{
  return b() + 2 * x * c() + 3 * d() * std::pow(x, 2);
}

double descartes_utilities::CubicSplineParams::acceleration(double x) const
{
  return 2 * c() + 6 * x * d();
}

double descartes_utilities::CubicSplineParams::xMaxVelocity() const
{
  return -1.0 / 3.0 * c() / d();
}

double descartes_utilities::CubicSplineParams::xMaxAcceleration() const
{
  return 6 * d();
}

/////////////////////////
// Spline Interpolator //
/////////////////////////
descartes_utilities::SplineInterpolator::SplineInterpolator(const std::vector<double>& x_data, 
                                                            const std::vector<double>& y_data,
                                                            double init_velocity, 
                                                            double final_velocity)
  : x_data_(x_data)
{
  // Asserts
  assert(x_data.size() > 1);
  assert(x_data.size() == y_data.size());
  // Memory efficiency
  spline_params_.reserve(x_data.size() - 1);
  
  const auto n = x_data.size();
  std::vector<double> yddot_data (n); // temporary storage for accelerations
  
  fit_splines(x_data, y_data, init_velocity, final_velocity, yddot_data);

  // Convert to spline parameters and save
  for (std::size_t i = 0; i < x_data_.size() - 1; ++i)
  {
    auto start = i;
    auto end = i + 1;
    double dt = x_data[end] - x_data[start];
    double a = y_data[start];
    double c = yddot_data[start] / 2.0;
    double d = (yddot_data[end] - yddot_data[start]) / (6 * dt);
    double b = (y_data[end] - y_data[start]) / dt - dt * (2*yddot_data[start] + yddot_data[end]) / 6;

    spline_params_.emplace_back(a,b,c,d);
  }
}

double descartes_utilities::SplineInterpolator::position(double x) const
{
  unsigned idx = this->search(x);
  return spline_params_[idx].position(x - x_data_[idx]);
}

double descartes_utilities::SplineInterpolator::velocity(double x) const
{
  unsigned idx = this->search(x);
  return spline_params_[idx].velocity(x - x_data_[idx]);
}

double descartes_utilities::SplineInterpolator::acceleration(double x) const
{
  unsigned idx = this->search(x);
  return spline_params_[idx].acceleration(x - x_data_[idx]);
}

std::pair<double, unsigned> descartes_utilities::SplineInterpolator::maxVelocity() const
{
  unsigned idx = 0;
  double max = 0.0;

  for (unsigned i = 0; i < spline_params_.size(); ++i)
  {
    double vel = maxSegmentVelocity(i);
    if (vel > max)
    {
      max = vel;
      idx = i;
    }
  }
  return {max, idx};
}

std::pair<double, unsigned> descartes_utilities::SplineInterpolator::maxAcceleration() const
{
  unsigned idx = 0;
  double max = 0.0;

  for (unsigned i = 0; i < spline_params_.size(); ++i)
  {
    double acc = maxSegmentAcceleration(i);
    if (acc > max)
    {
      max = acc;
      idx = i;
    }
  }
  return {max, idx};
}

unsigned descartes_utilities::SplineInterpolator::search(double x) const
{
  if (x <= x_data_.front()) return 0u;
  if (x >= x_data_.back()) return x_data_.size() - 2u;

  auto it = std::lower_bound(x_data_.begin(), x_data_.end(), x); // first point not less than x
  return std::distance(x_data_.begin(), it) - 1; // We want the previous point
}

double descartes_utilities::SplineInterpolator::maxSegmentVelocity(unsigned idx) const
{
  const auto x = spline_params_[idx].xMaxVelocity();
  const auto start_x = x_data_[idx];
  const auto stop_x = x_data_[idx+1];

  if (x > 0 && x < stop_x - start_x)
  {
    return spline_params_[idx].velocity(x);
  }
  else
  {
    const auto start_v = std::abs(spline_params_[idx].velocity(0.0));
    const auto stop_v = std::abs(spline_params_[idx].velocity(stop_x - start_x));
    return std::max(start_v, stop_v);
  }
}

double descartes_utilities::SplineInterpolator::maxSegmentAcceleration(unsigned idx) const
{
  const auto x = spline_params_[idx].xMaxAcceleration();
  const auto start_x = x_data_[idx];
  const auto stop_x = x_data_[idx+1];

  if (x > 0 && x < stop_x - start_x)
  {
    return spline_params_[idx].acceleration(x);
  }
  else
  {
    const auto start_a = std::abs(spline_params_[idx].acceleration(0.0));
    const auto stop_a = std::abs(spline_params_[idx].acceleration(stop_x - start_x));
    return std::max(start_a, stop_a);
  }
}
