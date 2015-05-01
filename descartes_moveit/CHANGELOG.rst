^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package descartes_moveit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2015-05-01)
------------------
* Calls the isValid(...) method in order to verify the joint solution
* Robot state now passed by reference to seed generator to remove requirement to use smart pointer. Added getter function for robot state object so you can get it even if you use the initialize method
* State adapter now automatically creates random seeds that can be mutated by calling helper functions in the seed namespace
Contributors: Jonathan Meyer, jrgnicho

0.0.2 (2015-04-09)
------------------
* Added install to CMakeLists.txt
* Contributors: Shaun Edwards

0.0.1 (2015-04-01)
------------------
* Alpha Release
* Contributors: Jonathan Meyer, Shaun Edwards, gavanderhoorn, jrgnicho, ros-devel
