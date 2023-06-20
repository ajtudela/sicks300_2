/*
 * SICK S300 2 ROS NODE
 *
 * Copyright (c) 2022-2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of sicks300_2 project.
 * 
 * All rights reserved.
 *
 */

#ifndef SICKS300_2__SICKS300_2_HPP_
#define SICKS300_2__SICKS300_2_HPP_

// C++
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
#include "common/ScannerSickS300.h"

typedef rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn rclcpp_CallReturn;

class SickS3002: public rclcpp_lifecycle::LifecycleNode{
	public:
		SickS3002(const std::string& name, bool intra_process_comms = false);
		~SickS3002();
		rclcpp_CallReturn on_configure(const rclcpp_lifecycle::State &);
		rclcpp_CallReturn on_activate(const rclcpp_lifecycle::State & state);
		rclcpp_CallReturn on_deactivate(const rclcpp_lifecycle::State & state);
		rclcpp_CallReturn on_cleanup(const rclcpp_lifecycle::State &);
		rclcpp_CallReturn on_shutdown(const rclcpp_lifecycle::State & state);

	private:
		rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
		rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr in_standby_pub_;
		rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
		std_msgs::msg::Bool in_standby_;
		rclcpp::Time synced_ros_time_;

		std::string frame_id_, scan_topic_, port_;
		int baud_, scan_id_;
		bool inverted_, debug_, synced_time_ready_;
		unsigned int synced_sick_stamp_;
		double scan_duration_, scan_cycle_time_, scan_delay_, communication_timeout_;
		ScannerSickS300 scanner_;
		rclcpp::TimerBase::SharedPtr timer_;

		bool open();
		bool receiveScan();
		void publishStandby(bool in_standby);
		void publishLaserScan(std::vector<double> vdDistM, std::vector<double> vdAngRAD, 
							std::vector<double> vdIntensAU, unsigned int iSickTimeStamp, 
							unsigned int iSickNow);
		void publishError(std::string error);
		void publishWarn(std::string warn);
};
#endif