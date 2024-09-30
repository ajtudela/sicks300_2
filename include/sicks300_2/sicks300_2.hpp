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

#ifndef SICKS300_2__SICKS300_2_HPP_
#define SICKS300_2__SICKS300_2_HPP_

// C++
#include <vector>
#include <string>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

// Common
#include "common/ScannerSickS300.hpp"

namespace sicks300_2
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @class sicks300_2::SickS300
 * @brief ROS2 driver for the SICK S300 Professional laser scanner
 */
class SickS300 : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief Construct a new Sick S300 object
   * @param options Node options
   */
  explicit SickS300(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the Sick S300 object
   */
  ~SickS300();

  /**
   * @brief Configure the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activate the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivate the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Cleanup the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Shutdown the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

protected:
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

  /**
   * @brief Open the scanner
   *
   * @return true if the scanner is opened
   */
  bool open();

  /**
   * @brief Receive the scan
   *
   * @return true if the scan is received
   */
  bool receiveScan();

  /**
   * @brief Publish the standby status
   *
   * @param in_standby Standby status
   */
  void publishStandby(bool in_standby);

  /**
   * @brief Publish the laser scan
   *
   * @param vdDistM Vector of distances in meters
   * @param vdAngRAD Vector of angles in radians
   * @param vdIntensAU Vector of intensities in arbitrary units
   * @param iSickTimeStamp Timestamp of the scan
   * @param iSickNow Current timestamp
   */
  void publishLaserScan(
    std::vector<double> vdDistM, std::vector<double> vdAngRAD,
    std::vector<double> vdIntensAU, unsigned int iSickTimeStamp, unsigned int iSickNow);

  /**
   * @brief Publish an error message
   *
   * @param error Error message
   */
  void publishError(std::string error);

  /**
   * @brief Publish a warning message
   *
   * @param warn Warning message
   */
  void publishWarn(std::string warn);

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr in_standby_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string frame_id_, scan_topic_, port_;
  int baud_, scan_id_;
  bool inverted_, debug_, synced_time_ready_;
  unsigned int synced_sick_stamp_;
  double scan_duration_, scan_cycle_time_, scan_delay_, communication_timeout_;
  std_msgs::msg::Bool in_standby_;
  rclcpp::Time synced_ros_time_;
  ScannerSickS300 scanner_;
};

}  // namespace sicks300_2

#endif  // SICKS300_2__SICKS300_2_HPP_
