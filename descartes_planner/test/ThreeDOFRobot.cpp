#include <descartes_trajectory_test/cartesian_robot.h>

class ThreeDOFRobot: public descartes_trajectory_test::CartesianRobot
{
public:
  ThreeDOFRobot():
    descartes_trajectory_test::CartesianRobot(0,0,3)
  {

  }

  virtual ~ThreeDOFRobot()
  {

  }

};