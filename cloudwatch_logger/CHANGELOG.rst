^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cloudwatch_logger
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.1 (2020-03-31)
------------------
* Bump package version to 3.0.1 (`#14 <https://github.com/aws-robotics/cloudwatchlogs-ros2/issues/14>`_)
* Fix linting issues found by clang-tidy 6.0 (`#12 <https://github.com/aws-robotics/cloudwatchlogs-ros2/issues/12>`_)
  * clang-tidy fixes
  * clang-tidy linting issues fixed manually
* Contributors: Miaofei Mei, Ryan Newell

3.0.0 (2019-09-06)
------------------
* remove changelog for new release (`#8 <https://github.com/aws-robotics/cloudwatchlogs-ros2/issues/8>`_)
* Bump version to 3.0.0, add CHANGELOG and missing dependencies
* add log node unit test (`#5 <https://github.com/aws-robotics/cloudwatchlogs-ros2/issues/5>`_)
  Signed-off-by: Miaofei <miaofei@amazon.com>
* Add build depend on std_srvs (`#4 <https://github.com/aws-robotics/cloudwatchlogs-ros2/issues/4>`_)
  * Add build depend on std_srvs
  * Add exec depend on std_srvs
* Rename ament_cmake_ros to ament_cmake in CMakeLists.txt
* initial ROS1 to ROS2 conversion
  [WIP] Ros1 conversion
* address PR comments
  Signed-off-by: Miaofei <miaofei@amazon.com>
* remove unnecessary files
  Signed-off-by: Miaofei <miaofei@amazon.com>
* add log_node_param_helper unit test
  Signed-off-by: Miaofei <miaofei@amazon.com>
* correct the information in the README and cleanup the CMakeLists.txt
  Signed-off-by: Miaofei <miaofei@amazon.com>
* workaround fastrtps typesupport crash
  Signed-off-by: Miaofei <miaofei@amazon.com>
* fix "ros2 launch" behavior
  Signed-off-by: Miaofei <miaofei@amazon.com>
* port cloudwatch_logger offline support to ros2
  Signed-off-by: Miaofei <miaofei@amazon.com>
* Convert all code, build files, launch files to ROS2.
* Source from aws-robotics/cloudwatchlogs-ros1 master branch
* Contributors: AAlon, Avishay Alon, Cameron Evans, M. M, Miaofei
