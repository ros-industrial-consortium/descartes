#include <descartes_trajectory/cart_trajectory_pt.h>

class TestPoint: public descartes_trajectory::CartTrajectoryPt
{
public:
  TestPoint(const std::vector<double>& joints)
  {
    vals_.resize(joints.size());
    vals_.assign(joints.begin(),joints.end());
  }

  virtual ~TestPoint()
  {

  }

  virtual bool getClosestJointPose(const std::vector<double> &seed_state,
                                     const RobotModel &model,
                                     std::vector<double> &joint_pose) const
  {
    joint_pose.clear();
    joint_pose.assign(vals_.begin(),vals_.end());
    return true;
  }

  virtual void getJointPoses(const RobotModel &model,
                                       std::vector<std::vector<double> > &joint_poses) const
  {
    joint_poses.clear();
    joint_poses.push_back(vals_);

  }

protected:

  std::vector<double> vals_;
};
