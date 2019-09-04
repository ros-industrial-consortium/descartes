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

using TracIKPtr = std::shared_ptr<TRAC_IK::TRAC_IK>;

using FloatT = float;
using EdgePropertiesF = descartes_planner::EdgeProperties<FloatT>;
typedef descartes_planner::PointSampleGroup<FloatT> PointSampleGroupF;
class ZAxixSampler: public descartes_planner::PointSampler<FloatT>
{
public:
  ZAxixSampler(const Eigen::Isometry3d& pose, moveit::core::RobotModelConstPtr model, TracIKPtr ik_solver,
               const std::vector<FloatT>& seed, std::size_t num_samples = 10):
     model_(model),
     ik_solver_(ik_solver),
     num_samples_(num_samples),
     seed_(seed)
  {

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
    return false;
  }

  PointSampleGroupF::Ptr getSamples() override
  {
    return nullptr;
  }

  PointSampleGroupF::Ptr getSample(std::size_t idx) override
  {
    return nullptr;
  }

  virtual ~ZAxixSampler()
  {

  }

protected:
  std::size_t num_samples_;
  std::vector<FloatT> seed_;
  TracIKPtr ik_solver_;
  moveit::core::RobotModelConstPtr model_;
};

class SpeedEvaluator: public descartes_planner::EdgeEvaluator<FloatT>
{
public:
  SpeedEvaluator(moveit::core::RobotModelConstPtr model, TracIKPtr ik_solver, float desired_tool_speed = 0.01):
    model_(model),
    ik_solver_(ik_solver),
    tool_speed_(desired_tool_speed)
  {

  }

  ~SpeedEvaluator()
  {

  }

  std::vector< EdgePropertiesF > evaluate(PointSampleGroupF::ConstPtr s1,
                                          PointSampleGroupF::ConstPtr s2) const
  {
    return {};
  }

protected:
  float tool_speed_;
  TracIKPtr ik_solver_;
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
  //TRAC_IK::TRAC_IK()
  return 0;
}

