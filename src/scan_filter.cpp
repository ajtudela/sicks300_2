// Copyright (c) 2022 Alberto J. Tudela Roldán
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanFilter : public rclcpp::Node
{
public:
  ScanFilter()
  : Node("scan_filter")
  {
    declare_parameter_if_not_declared(
      this, "lower_angle",
      rclcpp::ParameterValue(0.0), rcl_interfaces::msg::ParameterDescriptor()
      .set__description("The angle of the scan to begin filtering at"));
    this->get_parameter("lower_angle", lower_angle_);
    RCLCPP_INFO(
      this->get_logger(),
      "The parameter lower_angle is set to: %f", lower_angle_);

    declare_parameter_if_not_declared(
      this, "upper_angle",
      rclcpp::ParameterValue(0.0), rcl_interfaces::msg::ParameterDescriptor()
      .set__description("The angle of the scan to end filtering at"));
    this->get_parameter("upper_angle", upper_angle_);
    RCLCPP_INFO(
      this->get_logger(),
      "The parameter upper_angle is set to: %f", upper_angle_);


    // Create publisher and subscriber
    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      std::bind(&ScanFilter::scan_callback, this, std::placeholders::_1));
    laser_scan_filtered_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      "scan_filtered", rclcpp::SensorDataQoS());
  }

private:
  /**
   * @brief Declares static ROS2 parameter and sets it to a given value if it was not already declared.
   *
   * @param node A node in which given parameter to be declared
   * @param param_name The name of parameter
   * @param default_value Parameter value to initialize with
   * @param parameter_descriptor Parameter descriptor (optional)
  */
  template<typename NodeT>
  void declare_parameter_if_not_declared(
    NodeT node,
    const std::string & param_name,
    const rclcpp::ParameterValue & default_value,
    const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
    rcl_interfaces::msg::ParameterDescriptor())
  {
    if (!node->has_parameter(param_name)) {
      node->declare_parameter(param_name, default_value, parameter_descriptor);
    }
  }

  void scan_callback(const sensor_msgs::msg::LaserScan & msg)
  {
    // Create new message
    auto msg_filtered = sensor_msgs::msg::LaserScan();
    msg_filtered.ranges.resize(msg.ranges.size());
    msg_filtered.intensities.resize(msg.intensities.size());

    double start_angle = msg.angle_min;
    double current_angle = msg.angle_min;
    builtin_interfaces::msg::Time start_time = msg.header.stamp;
    unsigned int count = 0;

    // Loop through the scan and truncate the beginning and the end of the scan as necessary
    for (unsigned int i = 0; i < msg.ranges.size(); ++i) {
      // Wait until we get to our desired starting angle
      if (start_angle < lower_angle_) {
        start_angle += msg.angle_increment;
        current_angle += msg.angle_increment;
        // start_time.set__sec(start_time.sec + msg.time_increment);
      } else {
        msg_filtered.ranges[count] = msg.ranges[i];

        // Make sure  that we don't update intensity data if its not available
        if (msg.intensities.size() > i) {
          msg_filtered.intensities[count] = msg.intensities[i];
        }
        count++;

        // Check if we need to break out of the loop,
        // basically if the next increment will put us over the threshold
        if (current_angle + msg.angle_increment > upper_angle_) {
          break;
        }

        current_angle += msg.angle_increment;
      }
    }

    // Make sure to set all the needed fields on the filtered scan
    msg_filtered.header.frame_id = msg.header.frame_id;
    msg_filtered.header.stamp = start_time;
    msg_filtered.angle_min = start_angle;
    msg_filtered.angle_max = current_angle;
    msg_filtered.angle_increment = msg.angle_increment;
    msg_filtered.time_increment = msg.time_increment;
    msg_filtered.scan_time = msg.scan_time;
    msg_filtered.range_min = msg.range_min;
    msg_filtered.range_max = msg.range_max;

    msg_filtered.ranges.resize(count);

    if (msg.intensities.size() >= count) {
      msg_filtered.intensities.resize(count);
    }

    // Publish message
    laser_scan_filtered_pub_->publish(msg_filtered);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_filtered_pub_;

  float lower_angle_, upper_angle_;
};

/* Main */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;
  auto node = std::make_shared<ScanFilter>();
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
