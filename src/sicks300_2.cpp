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

// C++
#include <chrono>
#include <thread>

// ROS
#include "rclcpp/qos.hpp"
#include "nav2_util/node_utils.hpp"

#include "sicks300_2/sicks300_2.hpp"

using namespace std::chrono_literals;

SickS3002::SickS3002(const std::string& name, bool intra_process_comms) : 
					rclcpp_lifecycle::LifecycleNode(name, rclcpp::NodeOptions()
					.use_intra_process_comms(intra_process_comms)), 
					synced_ros_time_(this->now()), 
					synced_time_ready_(false), 
					synced_sick_stamp_(0){
}

SickS3002::~SickS3002(){
	if (timer_) {
		timer_->cancel();
		timer_.reset();
	}
}

rclcpp_CallReturn SickS3002::on_configure(const rclcpp_lifecycle::State &){
	RCLCPP_INFO(this->get_logger(), "Configuring the node...");

	// Declare and read parameters
	nav2_util::declare_parameter_if_not_declared(this, "port", rclcpp::ParameterValue("/dev/ttyUSB0"), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("USB port of the scanner"));
	this->get_parameter("port", port_);
	RCLCPP_INFO(this->get_logger(), "The parameter port is set to: %s", port_.c_str());

	nav2_util::declare_parameter_if_not_declared(this, "baud", rclcpp::ParameterValue(500000), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Baudrate to communicate with the laser scanner"));
	this->get_parameter("baud", baud_);
	RCLCPP_INFO(this->get_logger(), "The parameter baud is set to: %i", baud_);

	nav2_util::declare_parameter_if_not_declared(this, "scan_id", rclcpp::ParameterValue(7), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Identifier of the scanner"));
	this->get_parameter("scan_id", scan_id_);
	RCLCPP_INFO(this->get_logger(), "The parameter scan_id is set to: %i", scan_id_);

	nav2_util::declare_parameter_if_not_declared(this, "inverted", rclcpp::ParameterValue(false), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Option to invert the direction of the measurements"));
	this->get_parameter("inverted", inverted_);
	RCLCPP_INFO(this->get_logger(), "The parameter inverted is set to: %s", inverted_ ? "true" : "false");

	nav2_util::declare_parameter_if_not_declared(this, "scan_topic", rclcpp::ParameterValue("scan"), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("The topic where the laser scan will be published"));
	this->get_parameter("scan_topic", scan_topic_);
	RCLCPP_INFO(this->get_logger(), "The parameter scan_topic is set to: %s", scan_topic_.c_str());

	nav2_util::declare_parameter_if_not_declared(this, "frame_id", rclcpp::ParameterValue("base_laser_link"), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("The frame of the scanner"));
	this->get_parameter("frame_id", frame_id_);
	RCLCPP_INFO(this->get_logger(), "The parameter frame_id is set to: %s", frame_id_.c_str());

	nav2_util::declare_parameter_if_not_declared(this, "scan_duration", rclcpp::ParameterValue(0.025), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Time between laser scans"));
	this->get_parameter("scan_duration", scan_duration_);
	RCLCPP_INFO(this->get_logger(), "The parameter scan_duration is set to: %f", scan_duration_);

	nav2_util::declare_parameter_if_not_declared(this, "scan_cycle_time", rclcpp::ParameterValue(0.040), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Cycle time of the scan"));
	this->get_parameter("scan_cycle_time", scan_cycle_time_);
	RCLCPP_INFO(this->get_logger(), "The parameter scan_cycle_time is set to: %f", scan_cycle_time_);

	nav2_util::declare_parameter_if_not_declared(this, "debug", rclcpp::ParameterValue(false), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Option to toggle scanner debugging information"));
	this->get_parameter("debug", debug_);
	RCLCPP_INFO(this->get_logger(), "The parameter debug is set to: %s", debug_ ? "true" : "false");

	nav2_util::declare_parameter_if_not_declared(this, "communication_timeout", rclcpp::ParameterValue(0.2), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Timeout to shutdown the node"));
	this->get_parameter("communication_timeout", communication_timeout_);
	RCLCPP_INFO(this->get_logger(), "The parameter communication_timeout is set to: %f", communication_timeout_);

	// Read 'fields' params. Set 1 by default to be backwards compatible
	// TODO: Change this when ROS will support YAML mixed types
	ScannerSickS300::ParamType param;
	param.range_field = 1;
	nav2_util::declare_parameter_if_not_declared(this, "fields.1.scale", rclcpp::ParameterValue(0.01), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Scale of the field"));
	this->get_parameter("fields.1.scale", param.dScale);
	RCLCPP_INFO(this->get_logger(), "The parameter field.1.scale is set to: %f", param.dScale);

	nav2_util::declare_parameter_if_not_declared(this, "fields.1.start_angle", rclcpp::ParameterValue(-135.0 / 180.0 * M_PI), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Start angle of the field"));
	this->get_parameter("fields.1.start_angle", param.dStartAngle);
	RCLCPP_INFO(this->get_logger(), "The parameter field.1.start_angle is set to: %f", param.dStartAngle);

	nav2_util::declare_parameter_if_not_declared(this, "fields.1.stop_angle", rclcpp::ParameterValue(135.0 / 180.0 * M_PI), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Stop angle of the field"));
	this->get_parameter("fields.1.stop_angle", param.dStopAngle);
	RCLCPP_INFO(this->get_logger(), "The parameter field.1.stop_angle is set to: %f", param.dStopAngle);
	scanner_.setRangeField(1, param);

	// Configure the publishers
	laser_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, rclcpp::SensorDataQoS());
	in_standby_pub_ = this->create_publisher<std_msgs::msg::Bool>("scan_standby", rclcpp::QoS(1).transient_local());
	diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(1));

	// Open the laser scanner
	bool bOpenScan = this->open();
	if (!bOpenScan){
		RCLCPP_ERROR(this->get_logger(), "...scanner not available on port %s. Please, try again.", port_.c_str());
		return rclcpp_CallReturn::FAILURE;
	}else{
		// Wait for scan to get ready if successful
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		RCLCPP_INFO(this->get_logger(), "...scanner opened successfully on port %s", port_.c_str());
		
		return rclcpp_CallReturn::SUCCESS;
	}
}

rclcpp_CallReturn SickS3002::on_activate(const rclcpp_lifecycle::State &){
	RCLCPP_INFO(this->get_logger(), "Activating the node...");

	timer_ = this->create_wall_timer(std::chrono::duration<double>(scan_cycle_time_), std::bind(&SickS3002::receiveScan, this));

	// Explicitly activate the lifecycle publishers
	laser_scan_pub_->on_activate();
	in_standby_pub_->on_activate();
	diag_pub_->on_activate();

	return rclcpp_CallReturn::SUCCESS;
}

rclcpp_CallReturn SickS3002::on_deactivate(const rclcpp_lifecycle::State &){
	RCLCPP_INFO(this->get_logger(), "Deactivating the node...");

	if (timer_) {
		timer_->cancel();
		timer_.reset();
	}

	// Explicitly deactivate lifecycle publishers
	laser_scan_pub_->on_deactivate();
	in_standby_pub_->on_deactivate();
	diag_pub_->on_deactivate();

	return rclcpp_CallReturn::SUCCESS;
}

rclcpp_CallReturn SickS3002::on_cleanup(const rclcpp_lifecycle::State &){
	RCLCPP_INFO(this->get_logger(), "Cleaning the node...");

	// Release the shared pointers
	laser_scan_pub_.reset();
	in_standby_pub_.reset();
	diag_pub_.reset();
	timer_.reset();

	return rclcpp_CallReturn::SUCCESS;
}

rclcpp_CallReturn SickS3002::on_shutdown(const rclcpp_lifecycle::State & state){
	RCLCPP_INFO(this->get_logger(), "Shutdown the node from state %s.", state.label().c_str());

	// Release the shared pointers
	laser_scan_pub_.reset();
	in_standby_pub_.reset();
	diag_pub_.reset();
	timer_.reset();

	return rclcpp_CallReturn::SUCCESS;
}

bool SickS3002::open(){
	return scanner_.open(port_.c_str(), baud_, scan_id_);
}

bool SickS3002::receiveScan(){
	std::vector<double> ranges, rangeAngles, intensities;
	unsigned int iSickTimeStamp, iSickNow;

	int result = scanner_.getScan(ranges, rangeAngles, intensities, iSickTimeStamp, iSickNow, debug_);
	static rclcpp::Time pointTimeCommunicationOK(this->now());

	if (result){
		if (scanner_.isInStandby()){
			publishWarn("scanner in standby");
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 30, "scanner on port %s in standby", port_.c_str());
			publishStandby(true);
		}else{
			publishStandby(false);
			publishLaserScan(ranges, rangeAngles, intensities, iSickTimeStamp, iSickNow);
		}

		pointTimeCommunicationOK = this->now();
	}else{
		rclcpp::Duration diff(this->now() - pointTimeCommunicationOK);

		if (diff.seconds() > communication_timeout_){
			RCLCPP_WARN(this->get_logger(), "Communication timeout");
			return false;
		}
	}

	return true;
}

void SickS3002::publishStandby(bool in_standby){
	in_standby_.data = in_standby;
	in_standby_pub_->publish(in_standby_);
}

void SickS3002::publishLaserScan(std::vector<double> vdDistM, std::vector<double> vdAngRAD, std::vector<double> vdIntensAU, unsigned int iSickTimeStamp, unsigned int iSickNow){
	// Fill message
	int start_scan = 0;
	int num_readings = vdDistM.size(); // initialize with max scan size
	int stop_scan = vdDistM.size();

	// Sync handling: find out exact scan time by using the syncTime-syncStamp pair:
	// Timestamp: "This counter is internally incremented at each scan, i.e. every 40 ms (S300)"
	if (iSickNow != 0){
		synced_ros_time_ = this->now() - rclcpp::Duration::from_seconds(scan_cycle_time_);
		synced_sick_stamp_ = iSickNow;
		synced_time_ready_ = true;

		RCLCPP_DEBUG(this->get_logger(), "Got iSickNow, store sync-stamp: %d", synced_sick_stamp_);
	}else{
		synced_time_ready_ = false;
	}

	// Create LaserScan message
	sensor_msgs::msg::LaserScan laserScan;
	if (synced_time_ready_){
		double timeDiff = static_cast<int>(iSickTimeStamp - synced_sick_stamp_) * scan_cycle_time_;
		laserScan.header.stamp = synced_ros_time_ + rclcpp::Duration::from_seconds(timeDiff);

		RCLCPP_DEBUG(this->get_logger(), "Time::now() - calculated sick time stamp = %f",(this->now() - laserScan.header.stamp).seconds());
	}else{
		laserScan.header.stamp = this->now();
	}

	// Fill message
	laserScan.header.frame_id = frame_id_;
	laserScan.angle_increment = vdAngRAD[start_scan + 1] - vdAngRAD[start_scan];
	laserScan.range_min = 0.001;
	laserScan.range_max = 29.5; // though the specs state otherwise, the max range reported by the scanner is 29.96m
	laserScan.time_increment = (scan_duration_) / (vdDistM.size());

	// Rescale scan
	num_readings = vdDistM.size();
	laserScan.angle_min = vdAngRAD[start_scan]; // first ScanAngle
	laserScan.angle_max = vdAngRAD[stop_scan - 1]; // last ScanAngle
	laserScan.ranges.resize(num_readings);
	laserScan.intensities.resize(num_readings);

	// Check for inverted laser
	if (inverted_){
		// to be really accurate, we now invert time_increment
		// laserScan.header.stamp = rclcpp::Time(laserScan.header.stamp) + rclcpp::Duration::from_seconds(scanDuration_); //adding of the sum over all negative increments would be mathematically correct, but looks worse.
		laserScan.time_increment = - laserScan.time_increment;
	}else{
		laserScan.header.stamp = rclcpp::Time(laserScan.header.stamp) - rclcpp::Duration::from_seconds(scan_duration_); //to be consistent with the omission of the addition above
	}

	for (int i = 0; i < (stop_scan - start_scan); i++){
		if (inverted_){
			laserScan.ranges[i] = vdDistM[stop_scan-1-i];
			laserScan.intensities[i] = vdIntensAU[stop_scan-1-i];
		}else{
			laserScan.ranges[i] = vdDistM[start_scan + i];
			laserScan.intensities[i] = vdIntensAU[start_scan + i];
		}
	}

	// Publish Laserscan-message
	laser_scan_pub_->publish(laserScan);

	// Diagnostics
	diagnostic_msgs::msg::DiagnosticArray diagnostics;
	diagnostics.header.stamp = this->now();
	diagnostics.status.resize(1);
	diagnostics.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::OK;
	diagnostics.status[0].name = this->get_namespace();
	diagnostics.status[0].message = "sick scanner running";
	diag_pub_->publish(diagnostics);
}

void SickS3002::publishError(std::string error){
	diagnostic_msgs::msg::DiagnosticArray diagnostics;
	diagnostics.header.stamp = this->now();
	diagnostics.status.resize(1);
	diagnostics.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
	diagnostics.status[0].name = this->get_namespace();
	diagnostics.status[0].message = error;
	diag_pub_->publish(diagnostics);
}

void SickS3002::publishWarn(std::string warn){
	diagnostic_msgs::msg::DiagnosticArray diagnostics;
	diagnostics.header.stamp = this->now();
	diagnostics.status.resize(1);
	diagnostics.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
	diagnostics.status[0].name = this->get_namespace();
	diagnostics.status[0].message = warn;
	diag_pub_->publish(diagnostics);
}