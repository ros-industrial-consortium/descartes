#include "descartes_trajectory/axial_symmetric_pt.h"


namespace descartes_trajectory
{

AxialSymmetricPt::AxialSymmetricPt(double x, double y, double z,
                                   double rx, double ry, double rz,
                                   double orient_increment) :
  CartTrajectoryPt(TolerancedFrame(
                     descartes_core::utils::toFrame(x, y, z, rx, ry, rz,
                                                    descartes_core::utils::EulerConventions::XYZ),
                     ToleranceBase::zeroTolerance<PositionTolerance>(x, y, z), // Fixed position
                     // Fixed rx, ry, free rotation about z
                     ToleranceBase::createSymmetric<OrientationTolerance>(rx, ry, 0, 0, 0, 2.0 * M_PI)),
                   0.0, // The position discretization
                   orient_increment) // Orientation discretization (starting at -2Pi and marching to 2Pi)
{
}

} // end of namespace descartes_trajectory
