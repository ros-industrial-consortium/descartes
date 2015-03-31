#include "descartes_trajectory/axial_symmetric_pt.h"


using descartes_trajectory::TolerancedFrame;
using descartes_trajectory::AxialSymmetricPt;
using namespace descartes_core::utils;

static TolerancedFrame makeRotationalAxis(AxialSymmetricPt::FreeAxis axis)
{
  using namespace descartes_trajectory;

  Eigen::Affine3d rot = Eigen::Affine3d::Identity();
  PositionTolerance pos_tol = ToleranceBase::zeroTolerance<PositionTolerance>(0,0,0);
  OrientationTolerance orient_tol = ToleranceBase::createSymmetric<OrientationTolerance>(0.0, 0.0, 0.0, 
                                    ((axis == AxialSymmetricPt::X_AXIS) ? 2*M_PI : 0.0),
                                    ((axis == AxialSymmetricPt::Y_AXIS) ? 2*M_PI : 0.0),
                                    ((axis == AxialSymmetricPt::Z_AXIS) ? 2*M_PI : 0.0));
  return TolerancedFrame(rot, pos_tol, orient_tol);
} 


namespace descartes_trajectory
{

AxialSymmetricPt::AxialSymmetricPt(double x, double y, double z, double rx, double ry, double rz,
                                   double orient_increment, FreeAxis axis) :
  CartTrajectoryPt(toFrame(x, y, z, rx, ry, rz, EulerConventions::XYZ),
                   makeRotationalAxis(axis),
                   Eigen::Affine3d::Identity(),
                   Eigen::Affine3d::Identity(),
                   0.0, // The position discretization
                   orient_increment) // Orientation discretization (starting at -2Pi and marching to 2Pi)
{
}

AxialSymmetricPt::AxialSymmetricPt(const Eigen::Affine3d& pose, double orient_increment, FreeAxis axis) :
  CartTrajectoryPt(pose,
                   makeRotationalAxis(axis),
                   Eigen::Affine3d::Identity(),
                   Eigen::Affine3d::Identity(),
                   0.0, // The position discretization
                   orient_increment) // Orientation discretization (starting at -2Pi and marching to 2Pi)
{
}

} // end of namespace descartes_trajectory
