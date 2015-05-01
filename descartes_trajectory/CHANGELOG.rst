^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package descartes_trajectory
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2015-05-01)
------------------
* Initializes the wobj_pt tolerance frame member with a fully defined nominal pose
* Contributors: Shaun Edwards, jrgnicho

0.0.2 (2015-04-09)
------------------
* Created two new base class methods for trajectory points: clone and copy that preserve the prior semantics while making sure that the underlying data type is copied correctly
* Added install to CMakeLists.txt
* Merge pull request `#85 <https://github.com/ros-industrial-consortium/descartes/issues/85>`_ from `ros-industrial-consortium/issue_#84 <https://github.com/ros-industrial-consortium/issue_/issues/84>`__incorrect_cartesian_discretization
  Incorrect cartesian point discretization
* uses const references in all range-based for loops that used auto type deduction
* added computeCartesionPoses method which is invoked by any method that needs to discretize the cartesian pose such getCartesianPoses and getJointPoses
* Added line on main page to document axial symmetric pt
* Added documentation
* finds the closest pose in joint space when ik for the closest cartesian pose fails
* Contributors: Jonathan Meyer, Shaun Edwards, jrgnicho

0.0.1 (2015-04-01)
------------------
* Alpha Release
* Contributors: Jonathan Meyer, Shaun Edwards, gavanderhoorn, jrgnicho, ros-devel
