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

#include "rclcpp/rclcpp.hpp"
#include "sicks300_2/sicks300_2.hpp"

/* Main */
int main(int argc, char** argv){
	rclcpp::init(argc, argv);

	auto node = std::make_shared<SickS3002>("sicks300_2");

	while (rclcpp::ok()){
		bool bOpenScan = false;
		while (!bOpenScan && rclcpp::ok()){
			RCLCPP_INFO(node->get_logger(), "Opening scanner... (port:%s)", node->getPort().c_str());

			bOpenScan = node->open();

			// Check, if it is the first try to open scanner
			if (!bOpenScan){
				RCLCPP_ERROR(node->get_logger(), "...scanner not available on port %s. Will retry every second.", node->getPort().c_str());
				node->publishError("...scanner not available on port");
			}
			sleep(1); // wait for scan to get ready if successfull, or wait befor retrying
		}
		RCLCPP_INFO(node->get_logger(), "...scanner opened successfully on port %s", node->getPort().c_str());

		// Main loop
		while (rclcpp::ok()){
			// Read scan
			if (!node->receiveScan()){
				break;
			}
			rclcpp::spin_some(node);
		}
	}
	rclcpp::shutdown();
	return 0;
}
