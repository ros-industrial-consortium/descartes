/*
 * utils.h
 *
 *  Created on: Sep 10, 2019
 *      Author: jrgnicho
 */

#ifndef INCLUDE_DESCARTES_TESTS_UTILS_H_
#define INCLUDE_DESCARTES_TESTS_UTILS_H_

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

namespace descartes_test
{

using Pose3D = std::tuple<double,double,double,double,double,double>;

static geometry_msgs::Pose pose3DtoPoseMsg(const Pose3D& p)
{
  using namespace Eigen;
  geometry_msgs::Pose pose_msg;
  Eigen::Affine3d eigen_pose = Translation3d(Vector3d(std::get<0>(p),std::get<1>(p),std::get<2>(p))) *
      AngleAxisd(std::get<3>(p),Vector3d::UnitX()) * AngleAxisd(std::get<4>(p),Vector3d::UnitY()) *
      AngleAxisd(std::get<5>(p),Vector3d::UnitZ());

  tf::poseEigenToMsg(eigen_pose,pose_msg);
  return std::move(pose_msg);
}

static visualization_msgs::MarkerArray
toDottedLineMarker(const std::vector<geometry_msgs::PoseArray>& path, const std::string& frame_id,
                   const std::string& ns, const std::size_t start_id = 1,
                   const Pose3D& offset = std::make_tuple(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), const float line_width = 0.004,
                   const float point_size = 0.005)
{
  visualization_msgs::MarkerArray markers_msgs;
  visualization_msgs::Marker line_marker, points_marker;

  // configure line marker
  line_marker.action = line_marker.ADD;
  std::tie(line_marker.color.r, line_marker.color.g, line_marker.color.b, line_marker.color.a) =
      std::make_tuple(1.0, 1.0, 0.2, 1.0);
  line_marker.header.frame_id = frame_id;
  line_marker.type = line_marker.LINE_STRIP;
  line_marker.id = start_id;
  line_marker.lifetime = ros::Duration(0);
  line_marker.ns = ns;
  std::tie(line_marker.scale.x, line_marker.scale.y, line_marker.scale.z) = std::make_tuple(line_width, 0.0, 0.0);
  line_marker.pose = pose3DtoPoseMsg(offset);

  // configure point marker
  points_marker = line_marker;
  points_marker.type = points_marker.POINTS;
  points_marker.ns = ns;
  std::tie(points_marker.color.r, points_marker.color.g, points_marker.color.b, points_marker.color.a) =
      std::make_tuple(0.1, .8, 0.2, 1.0);
  std::tie(points_marker.scale.x, points_marker.scale.y, points_marker.scale.z) =
      std::make_tuple(point_size, point_size, point_size);

  int id_counter = start_id;
  for (auto& poses : path)
  {
    line_marker.points.clear();
    points_marker.points.clear();
    line_marker.points.reserve(poses.poses.size());
    points_marker.points.reserve(poses.poses.size());
    for (auto& pose : poses.poses)
    {
      geometry_msgs::Point p;
      std::tie(p.x, p.y, p.z) = std::make_tuple(pose.position.x, pose.position.y, pose.position.z);
      line_marker.points.push_back(p);
      points_marker.points.push_back(p);
    }

    line_marker.id = (++id_counter);
    points_marker.id = (++id_counter);
    markers_msgs.markers.push_back(line_marker);
    markers_msgs.markers.push_back(points_marker);
  }

  return markers_msgs;
}

static visualization_msgs::MarkerArray
toAxisPathMarker(const std::vector<geometry_msgs::PoseArray>& path, const std::string& frame_id, const std::string& ns,
                 const std::size_t start_id = 1, const double axis_scale = 0.001, const double axis_length = 0.03,
                 const Pose3D& offset = std::make_tuple(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
{
  using namespace Eigen;

  visualization_msgs::MarkerArray markers;

  auto create_line_marker = [&](const int id,
                                const std::tuple<float, float, float, float>& rgba) -> visualization_msgs::Marker {
    visualization_msgs::Marker line_marker;
    line_marker.action = line_marker.ADD;
    std::tie(line_marker.color.r, line_marker.color.g, line_marker.color.b, line_marker.color.a) = rgba;
    line_marker.header.frame_id = frame_id;
    line_marker.type = line_marker.LINE_LIST;
    line_marker.id = id;
    line_marker.lifetime = ros::Duration(0);
    line_marker.ns = ns;
    std::tie(line_marker.scale.x, line_marker.scale.y, line_marker.scale.z) = std::make_tuple(axis_scale, 0.0, 0.0);
    line_marker.pose = pose3DtoPoseMsg(offset);
    return std::move(line_marker);
  };

  // markers for each axis line
  int marker_id = start_id;
  visualization_msgs::Marker x_axis_marker = create_line_marker(++marker_id, std::make_tuple(1.0, 0.0, 0.0, 1.0));
  visualization_msgs::Marker y_axis_marker = create_line_marker(++marker_id, std::make_tuple(0.0, 1.0, 0.0, 1.0));
  visualization_msgs::Marker z_axis_marker = create_line_marker(++marker_id, std::make_tuple(0.0, 0.0, 1.0, 1.0));

  auto add_axis_line = [](const Isometry3d& eigen_pose, const Vector3d& dir, const geometry_msgs::Point& p1,
                          visualization_msgs::Marker& marker) {

    geometry_msgs::Point p2;
    Eigen::Vector3d line_endpoint;

    // axis endpoint
    line_endpoint = eigen_pose * dir;
    std::tie(p2.x, p2.y, p2.z) = std::make_tuple(line_endpoint.x(), line_endpoint.y(), line_endpoint.z());

    // adding line
    marker.points.push_back(p1);
    marker.points.push_back(p2);
  };

  for (auto& poses : path)
  {
    for (auto& pose : poses.poses)
    {
      Eigen::Isometry3d eigen_pose;
      tf::poseMsgToEigen(pose, eigen_pose);

      geometry_msgs::Point p1;
      std::tie(p1.x, p1.y, p1.z) = std::make_tuple(pose.position.x, pose.position.y, pose.position.z);

      add_axis_line(eigen_pose, Vector3d::UnitX() * axis_length, p1, x_axis_marker);
      add_axis_line(eigen_pose, Vector3d::UnitY() * axis_length, p1, y_axis_marker);
      add_axis_line(eigen_pose, Vector3d::UnitZ() * axis_length, p1, z_axis_marker);
    }
  }

  markers.markers.push_back(x_axis_marker);
  markers.markers.push_back(y_axis_marker);
  markers.markers.push_back(z_axis_marker);
  return std::move(markers);
}
}

#endif /* INCLUDE_DESCARTES_TESTS_UTILS_H_ */
