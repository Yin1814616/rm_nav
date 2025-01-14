set(_AMENT_PACKAGE_NAME "vehicle_simulator")
set(vehicle_simulator_VERSION "0.0.1")
set(vehicle_simulator_MAINTAINER "Ji Zhang <zhangji@cmu.edu>")
set(vehicle_simulator_BUILD_DEPENDS "rclcpp" "std_msgs" "nav_msgs" "sensor_msgs" "pcl_ros" "tf2" "tf2_ros" "tf2_geometry_msgs" "message_filters" "pcl_conversions" "gazebo_ros" "gazebo_msgs")
set(vehicle_simulator_BUILDTOOL_DEPENDS "ament_cmake")
set(vehicle_simulator_BUILD_EXPORT_DEPENDS "rclcpp" "std_msgs" "nav_msgs" "sensor_msgs" "pcl_ros" "tf2" "tf2_ros" "tf2_geometry_msgs" "message_filters" "pcl_conversions" "gazebo_ros" "gazebo_msgs")
set(vehicle_simulator_BUILDTOOL_EXPORT_DEPENDS )
set(vehicle_simulator_EXEC_DEPENDS "rclcpp" "std_msgs" "nav_msgs" "sensor_msgs" "pcl_ros" "tf2" "tf2_ros" "tf2_geometry_msgs" "message_filters" "pcl_conversions" "gazebo_ros" "gazebo_msgs")
set(vehicle_simulator_TEST_DEPENDS "ament_lint_auto" "ament_lint_common")
set(vehicle_simulator_GROUP_DEPENDS )
set(vehicle_simulator_MEMBER_OF_GROUPS )
set(vehicle_simulator_DEPRECATED "")
set(vehicle_simulator_EXPORT_TAGS)
list(APPEND vehicle_simulator_EXPORT_TAGS "<build_type>ament_cmake</build_type>")
list(APPEND vehicle_simulator_EXPORT_TAGS "<gazebo_ros gazebo_model_path=\"mesh\"/>")
