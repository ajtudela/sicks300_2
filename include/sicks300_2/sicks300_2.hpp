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
#include <boost/lexical_cast.hpp>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

// Common
#include "common/ScannerSickS300.h"

//#include <XmlRpcException.h>
//#define ROS_LOG_FOUND

class SickS3002: public rclcpp::Node{
	public:
		SickS3002(const std::string& name);
		~SickS3002();

		bool open();
		std::string getPort();
		bool receiveScan();
		void publishStandby(bool inStandby);
		void publishLaserScan(std::vector<double> vdDistM, std::vector<double> vdAngRAD, std::vector<double> vdIntensAU, unsigned int iSickTimeStamp, unsigned int iSickNow);
		void publishError(std::string error);
		void publishWarn(std::string warn);

	private:
		rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserScanPub_;
		rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr inStandbyPub_;
		rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagPub_;
		std_msgs::msg::Bool inStandby_;
		rclcpp::Time syncedROSTime_;

		std::string frameId_, port_;
		int baud_, scanId_;
		bool inverted_, debug_, syncedTimeReady_;
		unsigned int syncedSICKStamp_;
		double scanDuration_, scanCycleTime_, communicationTimeout_;
		ScannerSickS300 scanner_;
};
#endif