/*
 * test_graph_solver.cpp
 *
 *  Created on: Sep 4, 2019
 *      Author: jrgnicho
 */
#include <memory>
#include <descartes_planner/graph_solver.h>
#include <ros/node_handle.h>
#include <kdl_parser/kdl_parser.hpp>
#include <trac_ik/trac_ik.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_kdl.h>
#include <console_bridge/console.h>
#include <boost/format.hpp>
#include <eigen_conversions/eigen_msg.h>
#include "descartes_tests/utils.h"

using TracIKPtr = std::shared_ptr<TRAC_IK::TRAC_IK>;
using FKSolverPtr = std::shared_ptr<KDL::ChainFkSolverPos_recursive>;

using FloatT = float;
using Vector3T = Eigen::Matrix<FloatT, 3, 1>;
using VectorXT = Eigen::Matrix<FloatT, Eigen::Dynamic, 1>;
using IsometryT = Eigen::Transform<FloatT,3,Eigen::Isometry>;
using PointSamplerT = descartes_planner::PointSampler<FloatT>;
using PointSampleGroupT = descartes_planner::PointSampleGroup<FloatT>;
using EdgePropertiesF = descartes_planner::EdgeProperties<FloatT>;


static const double POS_TOLERANCE = 1e-4; // meters
static const double ROT_TOLERANCE = 1e-3; // radians
static const std::string ROBOT_MODEL_PARAM = "robot_description";
static const int MAX_ITERATIONS = 50;
static const double EPS = 1e-5;


EigenSTL::vector_Isometry3f generateTrajectory(const Eigen::Isometry3f& nominal_pose,
                                               const Eigen::Isometry3f& local_offset,
                                               const double& radius, const double& angle,
                                               const double& line_spacing, const std::size_t& num_lines,
                                               const std::size_t& num_points_per_line)
{
  using namespace Eigen;

  EigenSTL::vector_Isometry3f traj_poses;

  // determine sphere center
  Eigen::Isometry3f sphere_center = nominal_pose * Translation3f(0,0,-radius);  // moving by -radius along -z

  // computing increments
  const std::tuple<float,float> angle_params(-angle/2.0, (angle)/(num_lines - 1)); //(start, incr)
  const float width_ = (line_spacing * (num_lines - 1));
  const std::tuple<float,float> x_disp_params(-width_/2.0,line_spacing); //(start, incr)

  // creating center line first
  Eigen::Isometry3f new_waypoint;
  EigenSTL::vector_Isometry3f current_line;
  current_line.reserve(num_points_per_line);
  double current_angle = 0.0;
  double current_x_pos = 0.0;
  bool reverse_line = false;
  for(std::size_t l = 0; l < num_lines; l++)
  {
    current_x_pos = std::get<0>(x_disp_params) + l * std::get<1>(x_disp_params);
    current_angle = 0;
    current_line.clear();
    for(std::size_t i = 0; i < num_points_per_line; i++)
    {
      current_angle = std::get<0>(angle_params) + i * std::get<1>(angle_params);
      new_waypoint = AngleAxisf(current_angle,Vector3f::UnitX()) * Translation3f(current_x_pos,0,radius) * local_offset;
      current_line.push_back(new_waypoint);
    }

    if(reverse_line)
    {
      std::reverse(current_line.begin(),current_line.end());
    }
    reverse_line = !reverse_line;

    // applying transform
    std::for_each(current_line.begin(),current_line.end(),[&sphere_center](Eigen::Isometry3f& p){
      p = sphere_center * p;
    });

    // adding to trajectory
    traj_poses.insert(traj_poses.end(), current_line.begin(), current_line.end());
  }
  return std::move(traj_poses);
}

class ZAxixSampler: public PointSamplerT
{
public:
  ZAxixSampler(const IsometryT& pose, moveit::core::RobotModelConstPtr model, TracIKPtr ik_solver,
               const std::vector<FloatT>& seed, std::size_t num_samples = 10):
     pose_(pose),
     model_(model),
     ik_solver_(ik_solver),
     num_samples_(num_samples),
     seed_(seed),
     samples_(nullptr)
  {
    if(seed_.size() < getDofs())
    {
      std::string msg = boost::str(
          boost::format("Seed size of %1% is less than the dof size %2%") % seed_.size() % getDofs());
      throw std::runtime_error(msg);
    }

    if(getDofs()==0)
    {
      throw std::runtime_error("Kinematic chain is ill-formed");
    }
  }

  std::size_t getNumSamples() override
  {
    return num_samples_;
  }

  std::size_t getDofs() override
  {
    KDL::Chain chain;
    if(!ik_solver_->getKDLChain(chain))
    {
      CONSOLE_BRIDGE_logError("Failed to get KDL chain");
      return 0;
    }
    return chain.getNrOfJoints();
  }

  bool getSamples(PointSampleGroupT::Ptr g) override
  {
    // check if samples have been preallocated
    samples_ = samples_ == nullptr ? computeSamples() : samples_;
    if(samples_ == nullptr)
    {
      CONSOLE_BRIDGE_logError("Failed to compute samples for pose");
      g->num_samples = 0;
      return false;
    }
    *g = *samples_;
    return g->num_samples > 0;
  }

  PointSampleGroupT::Ptr getSamples() override
  {
    samples_ = samples_ == nullptr ? computeSamples() : samples_;
    if(samples_ == nullptr)
    {
      CONSOLE_BRIDGE_logError("Failed to compute samples for pose");
      return false;
    }
    PointSampleGroupT::Ptr samples = std::make_shared<PointSampleGroupT>(*samples_);
    return std::move(samples);
  }

  PointSampleGroupT::Ptr getSample(std::size_t idx) override
  {
    samples_ = samples_ == nullptr ? computeSamples() : samples_;
    if(samples_ == nullptr)
    {
      CONSOLE_BRIDGE_logError("Failed to compute samples for pose");
      return nullptr;
    }

    PointSampleGroupT::Ptr samples = std::make_shared<PointSampleGroupT>();
    std::size_t start_loc = idx * getDofs();
    std::size_t end_loc = (idx + 1 )* getDofs();
    if(end_loc > samples_->values.size())
    {
      CONSOLE_BRIDGE_logError("Requested Index %lu exceeds sample size %lu", idx, samples_->values.size());
      return nullptr;
    }
    samples->values.clear();
    samples->values.insert(samples->values.end(),samples_->values.begin() + start_loc,
                    samples_->values.begin() + end_loc);
    samples->num_dofs = getDofs();
    samples->num_samples = 1;

    return samples;
  }

  virtual ~ZAxixSampler()
  {

  }

protected:

  PointSampleGroupT::Ptr computeSamples()
  {
    using namespace Eigen;

    PointSampleGroupT::Ptr sample_group = std::make_shared<PointSampleGroupT>();

    // deconstructing pose
    Vector3T euler_angles = pose_.rotation().eulerAngles(0,1,2);
    Vector3T pos = pose_.translation();
    std::size_t buffer_size = getDofs() * num_samples_;
    if(buffer_size > 0)
    {
      sample_group->values.reserve(buffer_size);
      sample_group->num_dofs = getDofs();
      sample_group->num_samples = num_samples_;
    }
    else
    {
      CONSOLE_BRIDGE_logError("Got 0 buffer size for samples");
      return nullptr;
    }

    // setup intermediate result vars
    FloatT z_incr = (2 * M_PI)/num_samples_;
    std::vector<FloatT> sol(getDofs());
    KDL::JntArray joint_sol(getDofs());
    KDL::JntArray joint_seed(getDofs());
    joint_seed.data = Map<VectorXf>(&seed_[0],seed_.size()).cast<double>();

    // setup tolerance
    KDL::Twist tol(KDL::Vector(POS_TOLERANCE, POS_TOLERANCE, POS_TOLERANCE),
                   KDL::Vector(ROT_TOLERANCE,ROT_TOLERANCE,M_PI));


    IsometryT sampled_pose;
    KDL::Frame sampled_kdl_pose;
    std::size_t actual_num_samples = 0;
    for(std::size_t s = 0; s < num_samples_; s++)
    {
      sampled_pose = Translation3f(pos) * AngleAxisf(euler_angles[0],Vector3T::UnitX()) *
          AngleAxisf(euler_angles[1],Vector3T::UnitY()) * AngleAxisf(z_incr * s,Vector3T::UnitZ());
      tf::transformEigenToKDL(sampled_pose.cast<double>(),sampled_kdl_pose);
      int num_sol = ik_solver_->CartToJnt(joint_seed,sampled_kdl_pose,joint_sol, tol);
      if(num_sol == 0)
      {
        continue;
      }
      sol.resize(joint_sol.rows());
      VectorXT::Map(&sol[0],sol.size()) = joint_sol.data.cast<FloatT>();

      //sample_group->values.insert(sample_group->values.begin() + actual_num_samples * getDofs(),sol.begin(),sol.end());
      sample_group->values.insert(sample_group->values.end(),sol.begin(),sol.end());
      actual_num_samples += num_sol;
    }

    if(actual_num_samples == 0)
    {
      CONSOLE_BRIDGE_logError("0 solutions were found for the current waypoint");
      return nullptr;
    }
    else
    {
      //CONSOLE_BRIDGE_logInform("Found %lu samples",actual_num_samples);
    }

    sample_group->num_samples = actual_num_samples;
    return sample_group;
  }

  IsometryT pose_;
  std::size_t num_samples_;
  std::vector<FloatT> seed_;
  TracIKPtr ik_solver_;
  moveit::core::RobotModelConstPtr model_;
  PointSampleGroupT::Ptr samples_;
};

class SpeedEvaluator: public descartes_planner::EdgeEvaluator<FloatT>
{
public:
  SpeedEvaluator(moveit::core::RobotModelConstPtr model,
                 const std::vector<std::string>& joint_names,
                 TracIKPtr ik_solver,
                 std::array<FloatT,2> nominal_tool_speed = {0.01 /* m/s */, M_PI/10.0 /* rad/s */}):
    model_(model),
    joint_names_(joint_names),
    ik_solver_(ik_solver),
    tool_speed_(nominal_tool_speed)
  {
    ik_solver_->getKDLChain(kdl_chain_);
    fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
  }

  virtual ~SpeedEvaluator()
  {

  }

  std::vector< EdgePropertiesF > evaluate(PointSampleGroupT::ConstPtr s1,
                                          PointSampleGroupT::ConstPtr s2) const
  {
    using namespace Eigen;
    std::vector< EdgePropertiesF > edges;
    edges.reserve(s1->num_samples * s2->num_samples);
    EdgePropertiesF edge;
    KDL::JntArray jpos1(s1->num_dofs);
    KDL::JntArray jpos2(s2->num_dofs);
    std::vector<FloatT> jvals1(s1->num_dofs, 0.0), jvals2(s2->num_dofs,0.0);
    std::size_t idx1, idx2;
    std::array<FloatT,2> cart_time, cart_disp;
    for(std::size_t i1 = 0; i1 < s1->num_samples; i1++)
    {
      idx1 = i1 * s1->num_dofs;
      jvals1.assign(s1->values.begin() + idx1, s1->values.begin() + idx1 + s1->num_dofs);
      jpos1.data = Map<VectorXf>(&jvals1[0], jvals1.size()).cast<double>();
      for(std::size_t i2 = 0; i2 < s2->num_samples; i2++)
      {
        idx2 = i2 * s2->num_dofs;
        jvals2.assign(s2->values.begin() + idx2, s2->values.begin() + idx2 + s2->num_dofs);
        jpos2.data = Map<VectorXf>(&jvals2[0], jvals2.size()).cast<double>();

        // computing time
        cart_disp = computeCartDisplacement(jpos1, jpos2);
        cart_time[0] = cart_disp[0]/tool_speed_[0];
        cart_time[1] = cart_disp[1]/tool_speed_[1];


        edge.src_vtx.point_id = s1->point_id;
        edge.src_vtx.sample_index = i1;
        edge.dst_vtx.point_id = s2->point_id;
        edge.dst_vtx.sample_index = i2;
        edge.valid = true;
        Eigen::VectorXd diff = jpos1.data - jpos2.data;
        double sum = std::abs(diff.sum());
        edge.weight = (sum < 1e-6) ? 0 : diff.norm();

        double min_time = *std::min_element(cart_time.begin(),cart_time.end());
        if(!isWithinSpeedLimits(jpos1, jpos2, min_time))
        {
          CONSOLE_BRIDGE_logDebug("Velocity exceeded for points (%i: %lu, %i: %lu)",
                                  s1->point_id, i1, s2->point_id, i2);
          edge.valid = false;
        }
        else
        {
          edge.valid = true;
        }

        edges.push_back(edge);
      }
    }
    return std::move(edges);
  }

protected:

  std::array<FloatT,2> computeCartDisplacement(const KDL::JntArray& pos1, const KDL::JntArray& pos2) const
  {
    using namespace Eigen;
    std::array<FloatT,2> cart_dplc;
    KDL::Frame c1, c2;
    Isometry3d pose1, pose2;
    fk_solver_->JntToCart(pos1,c1);
    fk_solver_->JntToCart(pos2,c2);

    tf::transformKDLToEigen(c1, pose1);
    tf::transformKDLToEigen(c2, pose2);
    Vector3d pos_delta = pose2.translation() - pose1.translation();
    AngleAxisd rot_delta(pose1.rotation().inverse() * pose2.rotation());

    std::tie(cart_dplc[0], cart_dplc[1]) = std::make_tuple(pos_delta.norm(),rot_delta.angle());
    return std::move(cart_dplc);
  }

  bool isWithinSpeedLimits(const KDL::JntArray& pos1, const KDL::JntArray& pos2, double t) const
  {
    using namespace Eigen;
    VectorXd diff = pos2.data - pos1.data;
    if(pos2.data.rows() != joint_names_.size() || pos1.data.rows() != joint_names_.size() )
    {
      CONSOLE_BRIDGE_logError("Size of joint position %i differs from passed joint names %lu",pos2.data.rows(),
                              joint_names_.size());
      return false;
    }

    for(std::size_t i = 0; i < diff.rows(); i++)
    {
      //CONSOLE_BRIDGE_logInform("Getting joint %s bounds",joint_names_[i].c_str());
      const moveit::core::VariableBounds& bounds = model_->getVariableBounds(joint_names_[i]);
      if(!bounds.velocity_bounded_)
      {
        continue;
      }

      double joint_vel = diff[i]/t;
      if(bounds.max_velocity_ < joint_vel)
      {
        return false;
      }
    }
    return true;
  }

  KDL::Chain kdl_chain_;
  std::vector<std::string> joint_names_;
  mutable FKSolverPtr fk_solver_;
  std::array<FloatT,2> tool_speed_;
  mutable TracIKPtr ik_solver_;
  moveit::core::RobotModelConstPtr model_;

};

class DescartesGraphPlanner
{
public:
  DescartesGraphPlanner(descartes_planner::EdgeEvaluator<FloatT>::Ptr edge_evaluator):
    solver_(edge_evaluator)
  {

  }

  virtual ~DescartesGraphPlanner()
  {

  }

  bool plan(std::vector<PointSamplerT::Ptr>& samplers, std::vector<PointSampleGroupT::ConstPtr>& sol)
  {
    if(!solver_.build(samplers))
    {
      ROS_ERROR("Failed to build graph");
      return false;
    }

    if(!solver_.solve(sol))
    {
      ROS_ERROR("Failed to solve graph");
      return false;
    }

    return true;
  }


protected:
  descartes_planner::GraphSolver<FloatT> solver_;

};

int main(int argc, char** argv)
{
  ros::init(argc,argv,"test_descartes_graph_solver");
  ros::NodeHandle nh, ph1("~/robot_info"), ph2("~/traj_generation");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // ros comm
  ros::Publisher traj_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("descartes_test_traj",1);


  // loading required robot parameters
  std::string base_link, tip_link, group_name;
  std::vector<double> reference_joint_pose;
  if(!ph1.getParam("base_link",base_link) ||
      !ph1.getParam("tip_link",tip_link) ||
      !ph1.getParam("group_name",group_name) ||
      !ph1.getParam("reference_joint_pose", reference_joint_pose))
  {
    ROS_ERROR("Failed to load robot_info parameters");
    return -1;
  }

  // loading required trajectory parameters
  std::vector<double> nominal_pose_vals, local_offset_vals;
  std::vector<int> grid_lines; // num lines in x and y
  std::vector<double> grid_dims; // lenth (x) and width (y)
  double curvature;
  if((!ph2.getParam("grid_dims",grid_dims) ||
      !ph2.getParam("grid_lines",grid_lines) ||
      !ph2.getParam("nominal_pose",nominal_pose_vals) ||
      !ph2.getParam("local_offset",local_offset_vals) ||
      !ph2.getParam("curvature",curvature) ))
  {
    ROS_ERROR("Failed to load traj_generation parameters");
    return -1;
  }

  auto vecToIsometry3f = [](const std::vector<double>& v) -> Eigen::Isometry3f {
    using namespace Eigen;
    Eigen::Isometry3f p = Translation3f(v[0],v[1],v[2]) * AngleAxisf(v[3],Vector3f::UnitX())
        * AngleAxisf(v[4],Vector3f::UnitY()) * AngleAxisf(v[5],Vector3f::UnitZ()) ;
    return std::move(p);
  };
  Eigen::Isometry3f nominal_pose = vecToIsometry3f(nominal_pose_vals);
  Eigen::Isometry3f local_offset = vecToIsometry3f(local_offset_vals);
  double radius = grid_dims[1]/curvature;
  double angle = 2 * std::asin(curvature/2.0);
  double line_spacing = grid_dims[0]/grid_lines[0];
  int num_lines = grid_lines[0];
  int num_points_per_line = grid_lines[1];

  EigenSTL::vector_Isometry3f traj_waypoints = generateTrajectory(nominal_pose, local_offset, radius, angle,
                                                 line_spacing, num_lines, num_points_per_line);

  // loading urdf
  robot_model_loader::RobotModelLoader model_loader(ROBOT_MODEL_PARAM,false);

  // loading solver
  TracIKPtr ik_solver = std::make_shared<TRAC_IK::TRAC_IK>(base_link,
                                                             tip_link,
                                                             ROBOT_MODEL_PARAM);
  std::vector<std::string> joint_names;
  moveit::core::JointModelGroup* group = model_loader.getModel()->getJointModelGroup(group_name);
  // creating planner now
  SpeedEvaluator::Ptr speed_eval = std::make_shared<SpeedEvaluator>(model_loader.getModel(),
                                                                    group->getActiveJointModelNames(),
                                                                    ik_solver);
  DescartesGraphPlanner planner(speed_eval);

  // visualizing trajectory
  std::string world_frame = model_loader.getModel()->getRootLinkName();
  geometry_msgs::PoseArray traj_poses;
  std::transform(traj_waypoints.begin(),traj_waypoints.end(),std::back_inserter(traj_poses.poses),[](
      const decltype(traj_waypoints)::value_type& w){
    geometry_msgs::Pose pose;
    tf::poseEigenToMsg(w.cast<double>(),pose);
    return std::move(pose);
  });

  visualization_msgs::MarkerArray traj_line_markers = descartes_test::toDottedLineMarker({traj_poses},world_frame,"lines");
  visualization_msgs::MarkerArray traj_axis_markers = descartes_test::toAxisPathMarker({traj_poses},world_frame,"axis");

  ros::Timer publish_timer = nh.createTimer(ros::Duration(1.0),[&](const ros::TimerEvent& e){
    traj_marker_pub.publish(traj_line_markers);
    traj_marker_pub.publish(traj_axis_markers);
  });

  // create input trajectory
  std::vector<PointSamplerT::Ptr> samplers;
  std::vector<FloatT> seed_pose;
  std::transform(reference_joint_pose.begin(), reference_joint_pose.end(),std::back_inserter(seed_pose),[](const double& v){
    return static_cast<float>(v);
  });
  std::transform(traj_waypoints.begin(),traj_waypoints.end(),std::back_inserter(samplers),[&](const Eigen::Isometry3f& wp){
    ZAxixSampler::Ptr sampler = std::make_shared<ZAxixSampler>(wp,model_loader.getModel(),ik_solver,seed_pose);
    return sampler;
  });


  std::vector<PointSampleGroupT::ConstPtr> sol;
  if(!planner.plan(samplers,sol))
  {
   return -1;
  }
  ROS_INFO_STREAM("Found solution");

  ros::waitForShutdown();

  return 0;
}

