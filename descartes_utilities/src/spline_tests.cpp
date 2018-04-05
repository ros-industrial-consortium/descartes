#include <iostream>

#include "descartes_utilities/spline_interpolator.h"
#include "descartes_utilities/ros_conversions.h"

#include <fstream>

int main()
{
  // using namespace trajectory_msgs;
  // std::vector<JointTrajectoryPoint> points;

  // for (std::size_t i = 0; i < 10; i++)
  // {
  //   auto pt = JointTrajectoryPoint();
  //   pt.positions.resize(6, i / 10.0);
  //   pt.time_from_start = ros::Duration(i);
  //   points.push_back(pt);
  // }

  // if (descartes_utilities::parameterizeVelocityAcceleration(points))
  // {
  //   std::cout << "It worked\n";

  //   std::ofstream j1 ("j1.csv");
  //   std::ofstream j2 ("j2.csv");
  //   std::ofstream j3 ("j3.csv");
  //   std::ofstream j4 ("j4.csv");
  //   std::ofstream j5 ("j5.csv");
  //   std::ofstream j6 ("j6.csv");

  //   for (auto&& point : points)
  //   {
  //     auto tm = point.time_from_start.toSec();
  //     j1 << tm << ",";
  //     j2 << tm << ",";
  //     j3 << tm << ",";
  //     j4 << tm << ",";
  //     j5 << tm << ",";
  //     j6 << tm << ",";

  //     j1 << point.positions[0] << "," << point.velocities[0] << "," << point.accelerations[0] << "\n";
  //     j2 << point.positions[1] << "," << point.velocities[1] << "," << point.accelerations[1] << "\n";
  //     j3 << point.positions[2] << "," << point.velocities[2] << "," << point.accelerations[2] << "\n";
  //     j4 << point.positions[3] << "," << point.velocities[3] << "," << point.accelerations[3] << "\n";
  //     j5 << point.positions[4] << "," << point.velocities[4] << "," << point.accelerations[4] << "\n";
  //     j6 << point.positions[5] << "," << point.velocities[5] << "," << point.accelerations[5] << "\n";
  //   }

  //   std::cout << "Ji: " << points.front().positions[0] << " " << points.front().velocities[0] << " " << points.front().accelerations[0] << "\n";
  //   std::cout << "Jf: " << points.back().positions[0] << " " << points.back().velocities[0] << " " << points.back().accelerations[0] << "\n";
  // }
  // else
  // {
  //   std::cout << "It did not work\n";
  // }


  
  std::vector<double> x = {0,1,2,3,4,5,6,7,8,9,10};
  std::vector<double> y = {0,1,2,3,4,6,3, 1, -1, -2, 3};

  descartes_utilities::SplineInterpolator spline (x, y, 0.0, 0.0);

  std::ofstream of ("test.csv");

  for (double x = 0.0; x < 10.1; x += 0.1)
  {
    of << x << "," << spline.position(x) 
       << "," << spline.velocity(x) << "," << spline.acceleration(x) << "\n";
  }

  auto max_vel = spline.maxVelocity();
  auto max_acc = spline.maxAcceleration();

  std::cout << "Spline segments: " << spline.splineParams().size() << "\n";

  std::cout << "Max velocity: " << max_vel.first << " at " << max_vel.second << '\n';
  std::cout << "Max accleration: " << max_acc.first << " at " << max_acc.second << '\n';
  
}