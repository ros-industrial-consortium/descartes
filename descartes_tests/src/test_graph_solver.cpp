/*
 * test_graph_solver.cpp
 *
 *  Created on: Sep 4, 2019
 *      Author: jrgnicho
 */
#include <memory>
#include <descartes_planner/graph_solver.h>
#include <ros/node_handle.h>
#include <trac_ik/utils.h>
#include <trac_ik/trac_ik.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_kdl.h>
#include <console_bridge/console.h>
#include <boost/format.hpp>

using TracIKPtr = std::shared_ptr<TRAC_IK::TRAC_IK>;
using FKSolverPtr = std::shared_ptr<KDL::ChainFkSolverPos_recursive>;

using FloatT = float;
using Vector3T = Eigen::Matrix<FloatT, 3, 1>;
using VectorXT = Eigen::Matrix<FloatT, Eigen::Dynamic, 1>;
using IsometryT = Eigen::Transform<FloatT,3,Eigen::Isometry>;
using EdgePropertiesF = descartes_planner::EdgeProperties<FloatT>;
typedef descartes_planner::PointSampleGroup<FloatT> PointSampleGroupF;


static const double POS_TOLERANCE = 1e-4; // meters
static const double ROT_TOLERANCE = 1e-3; // radians

class ZAxixSampler: public descartes_planner::PointSampler<FloatT>
{
public:
  ZAxixSampler(const IsometryT& pose, moveit::core::RobotModelConstPtr model, TracIKPtr ik_solver,
               const std::vector<FloatT>& seed, std::size_t num_samples = 10):
     pose_(pose),
     model_(model),
     ik_solver_(ik_solver),
     num_samples_(num_samples),
     seed_(seed)
  {
    if(seed_.size() < getDofs())
    {
      std::string msg = boost::str(
          boost::format("Seed size of %1% is less than the dof size %2%") % seed_.size() % getDofs());
      throw std::runtime_error(msg);
    }
  }

  std::size_t getNumSamples() override
  {
    return num_samples_;
  }

  std::size_t getDofs() override
  {
    return ik_solver_->getLowerLimits().rows();
  }

  bool getSamples(PointSampleGroupF::Ptr g) override
  {
    // check if samples have been preallocated
    samples_ = samples_ == nullptr ? computeSamples() : nullptr;
    if(samples_ == nullptr)
    {
      CONSOLE_BRIDGE_logError("Failed to compute samples for pose");
      return false;
    }
    *g = *samples_;
    return true;
  }

  PointSampleGroupF::Ptr getSamples() override
  {
    samples_ = samples_ == nullptr ? computeSamples() : nullptr;
    if(samples_ == nullptr)
    {
      CONSOLE_BRIDGE_logError("Failed to compute samples for pose");
      return false;
    }
    PointSampleGroupF::Ptr samples = std::make_shared<PointSampleGroupF>(*samples_);
    return std::move(samples);
  }

  PointSampleGroupF::Ptr getSample(std::size_t idx) override
  {
    samples_ = samples_ == nullptr ? computeSamples() : nullptr;
    if(samples_ == nullptr)
    {
      CONSOLE_BRIDGE_logError("Failed to compute samples for pose");
      return false;
    }

    PointSampleGroupF::Ptr samples = std::make_shared<PointSampleGroupF>();
    std::size_t start_loc = idx * getDofs();
    std::size_t end_loc = (idx + 1 * getDofs());
    if(end_loc > samples_->values.size())
    {
      CONSOLE_BRIDGE_logError("Requested Index %lu exceeds sample size %lu", idx, samples_->values.size());
      return nullptr;
    }
    samples->values.insert(samples->values.begin(),samples_->values.begin() + start_loc,
                    samples_->values.begin() + end_loc);
    samples->num_dofs = getDofs();
    samples->num_samples = 1;

    return samples;
  }

  virtual ~ZAxixSampler()
  {

  }

protected:

  PointSampleGroupF::Ptr computeSamples()
  {
    using namespace Eigen;

    PointSampleGroupF::Ptr sample_group = std::make_shared<PointSampleGroupF>();

    // deconstructing pose
    Vector3T euler_angles = pose_.rotation().eulerAngles(0,1,2);
    Vector3T pos = pose_.translation();
    std::size_t buffer_size = getDofs() * num_samples_;
    if(buffer_size > sample_group->values.size())
    {
      sample_group->values.resize(buffer_size);
      sample_group->num_dofs = getDofs();
      sample_group->num_samples = num_samples_;
    }

    // setup intermediate result vars
    FloatT z_incr = (2 * M_PI)/num_samples_;
    std::vector<FloatT> sol;
    KDL::JntArray joint_sol;
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

      sample_group->values.insert(sample_group->values.begin() + actual_num_samples * getDofs(),sol.begin(),sol.end());
      actual_num_samples += num_sol;
    }

    if(actual_num_samples == 0)
    {
      return nullptr;
    }

    sample_group->num_samples = actual_num_samples;
    return sample_group;
  }

  IsometryT pose_;
  std::size_t num_samples_;
  std::vector<FloatT> seed_;
  TracIKPtr ik_solver_;
  moveit::core::RobotModelConstPtr model_;
  PointSampleGroupF::Ptr samples_;
};

class SpeedEvaluator: public descartes_planner::EdgeEvaluator<FloatT>
{
public:
  SpeedEvaluator(moveit::core::RobotModelConstPtr model,
                 std::vector<std::string>& joint_names,
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

  std::vector< EdgePropertiesF > evaluate(PointSampleGroupF::ConstPtr s1,
                                          PointSampleGroupF::ConstPtr s2) const
  {
    using namespace Eigen;
    std::vector< EdgePropertiesF > valid_edges;
    valid_edges.reserve(s1->num_samples * s2->num_samples);
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

        double min_time = *std::min_element(cart_time.begin(),cart_time.end());
        if(!isWithinSpeedLimits(jpos1, jpos2, min_time))
        {
          CONSOLE_BRIDGE_logDebug("Velocity exceeded for points (%i: %lu, %i: %lu)",
                                  s1->point_id, i1, s2->point_id, i2);
          continue;
        }

        edge.src_vtx.point_id = s1->point_id;
        edge.src_vtx.sample_index = i1;
        edge.dst_vtx.point_id = s2->point_id;
        edge.dst_vtx.sample_index = i2;
        edge.valid = true;
        edge.weight = (jpos1.data - jpos2.data).norm();
        valid_edges.push_back(edge);
      }
    }
    return {};
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

    for(std::size_t i = 0; i < diff.rows(); i++)
    {
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


protected:
  descartes_planner::GraphSolver<FloatT> solver_;

};


int main(int argc, char** argv)
{
  ros::init(argc,argv,"test_descartes_graph_solver");
  ros::NodeHandle ph("~");

  // loading required parameters
  std::string base_link, tip_link;
  if(!ph.getParam("base_link",base_link) ||
      !ph.getParam("tip_link",tip_link))
  {
    ROS_ERROR("Failed to load parameters");
    return 1;
  }

  // loading urdf
  static const std::string ROBOT_MODEL_PARAM = "robot_description";
  static const int MAX_ITERATIONS = 50;
  static const double EPS = 1e-5;
  robot_model_loader::RobotModelLoader model_loader(ROBOT_MODEL_PARAM,false);

  // loading solver
  ros::NodeHandle nh;
  KDL::Chain chain;
  std::vector<std::string> link_names, joint_names;
  KDL::JntArray joint_min, joint_max;
  urdf::ModelSharedPtr urdf_model = boost::make_shared<urdf::Model>();
  if(!TRAC_IK::LoadModelOverride(nh, ROBOT_MODEL_PARAM, *urdf_model))
  {
    ROS_WARN_STREAM_NAMED("trac_ik", "Failed to load robot model");
    return -1;
  }

  if (!TRAC_IK::InitKDLChain( *urdf_model, base_link, tip_link,chain,
                              link_names, joint_names, joint_min, joint_max))
  {
    ROS_WARN_STREAM_NAMED("trac_ik", "Failed to initialize KDL chain");
    return -1;
  }

  TracIKPtr ik_solver = std::make_shared<TRAC_IK::TRAC_IK>(chain,
                                                           joint_min,
                                                           joint_max,
                                                           MAX_ITERATIONS, EPS, TRAC_IK::Speed);
  // creating planner now
  SpeedEvaluator::Ptr speed_eval = std::make_shared<SpeedEvaluator>(model_loader.getModel(), joint_names,
                                                                    ik_solver);
  DescartesGraphPlanner planner(speed_eval);

  return 0;
}

