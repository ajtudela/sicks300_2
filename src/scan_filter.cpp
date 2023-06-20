/*
 * SCAN FILTER ROS NODE
 *
 * Copyright (c) 2022-2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of sicks300_2 project.
 * 
 * All rights reserved.
 *
 */

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "nav2_util/node_utils.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanFilter: public rclcpp::Node{
	public:
		ScanFilter(): Node("scan_filter"){
			nav2_util::declare_parameter_if_not_declared(this, "lower_angle", 
				rclcpp::ParameterValue(0), rcl_interfaces::msg::ParameterDescriptor()
						.set__description("The angle of the scan to begin filtering at"));
			this->get_parameter("lower_angle", lower_angle_);
			RCLCPP_INFO(this->get_logger(), 
				"The parameter lower_angle is set to: %f", lower_angle_);
			
			nav2_util::declare_parameter_if_not_declared(this, "upper_angle", 
				rclcpp::ParameterValue(0), rcl_interfaces::msg::ParameterDescriptor()
						.set__description("The angle of the scan to end filtering at"));
			this->get_parameter("upper_angle", upper_angle_);
			RCLCPP_INFO(this->get_logger(), 
				"The parameter upper_angle is set to: %f", upper_angle_);


			// Create publisher and subscriber
			laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
				"scan", rclcpp::SensorDataQoS(), 
				std::bind(&ScanFilter::scan_callback, this, std::placeholders::_1));
			laser_scan_filtered_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
				"scan_filtered", rclcpp::SensorDataQoS());
		}

	private:
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
		rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_filtered_pub_;

		float lower_angle_, upper_angle_;

		void scan_callback(const sensor_msgs::msg::LaserScan &msg){
			// Create new message
			auto msg_filtered = sensor_msgs::msg::LaserScan();
			msg_filtered.ranges.resize(msg.ranges.size());
			msg_filtered.intensities.resize(msg.intensities.size());

			double start_angle = msg.angle_min;
			double current_angle = msg.angle_min;
			builtin_interfaces::msg::Time start_time = msg.header.stamp;
			unsigned int count = 0;

			// Loop through the scan and truncate the beginning and the end of the scan as necessary
			for (unsigned int i = 0; i < msg.ranges.size(); ++i){
				// Wait until we get to our desired starting angle
				if ( start_angle < lower_angle_){
					start_angle += msg.angle_increment;
					current_angle += msg.angle_increment;
					//start_time.set__sec(start_time.sec + msg.time_increment);
				}else{
					msg_filtered.ranges[count] = msg.ranges[i];

					// Make sure  that we don't update intensity data if its not available
					if (msg.intensities.size() > i){
						msg_filtered.intensities[count] = msg.intensities[i];
					}
					count++;

					// Check if we need to break out of the loop, 
					// basically if the next increment will put us over the threshold
					if (current_angle + msg.angle_increment > upper_angle_){
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

			if (msg.intensities.size() >= count){
				msg_filtered.intensities.resize(count);
			}

			// Publish message
			laser_scan_filtered_pub_->publish(msg_filtered);
		}
};

/* Main */
int main(int argc, char** argv){
	rclcpp::init(argc, argv);

	rclcpp::executors::SingleThreadedExecutor exe;
	auto node = std::make_shared<ScanFilter>();
	exe.add_node(node->get_node_base_interface());
	exe.spin();
	rclcpp::shutdown();
	return 0;
}

