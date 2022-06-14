/*
 * SICK S300 2 ROS NODE
 *
 * Copyright (c) 2022 Alberto José Tudela Roldán <ajtudela@gmail.com>
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

// BOOST
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

// Common
#include "common/ScannerSickS300.h"

class SickS3002: public rclcpp::Node{
	public:
		SickS3002(const std::string& name);
		~SickS3002();

		bool open();
		std::string getPort();
		bool receiveScan();
		void publishStandby(bool in_standby);
		void publishLaserScan(std::vector<double> vdDistM, std::vector<double> vdAngRAD, std::vector<double> vdIntensAU, unsigned int iSickTimeStamp, unsigned int iSickNow);
		void publishError(std::string error);
		void publishWarn(std::string warn);

	private:
		rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
		rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr in_standby_pub_;
		rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
		std_msgs::msg::Bool in_standby_;
		rclcpp::Time synced_ros_time_;

		std::string frame_id_, scan_topic_, port_;
		int baud_, scan_id_;
		bool inverted_, debug_, synced_time_ready_;
		unsigned int synced_sick_stamp_;
		double scan_duration_, scan_cycle_time_, communication_timeout_;
		ScannerSickS300 scanner_;
};
#endif