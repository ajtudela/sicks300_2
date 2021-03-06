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

#include "rclcpp/qos.hpp"
#include "sicks300_2/sicks300_2.hpp"

SickS3002::SickS3002(const std::string& name, bool intra_process_comms) : 
					rclcpp_lifecycle::LifecycleNode(name, rclcpp::NodeOptions()
					.use_intra_process_comms(intra_process_comms)
					.automatically_declare_parameters_from_overrides(true)), 
					synced_ros_time_(this->now()), 
					synced_time_ready_(false), 
					synced_sick_stamp_(0){
}

SickS3002::~SickS3002(){
}

rclcpp_CallReturn SickS3002::on_configure(const rclcpp_lifecycle::State &){
	RCLCPP_INFO(this->get_logger(), "Configuring the node...");

	// Initialize node
	if (!this->has_parameter(("port"))){
		RCLCPP_WARN(this->get_logger(), "Used default parameter for port");
		port_ = this->declare_parameter("port", std::string("/dev/ttyUSB0"));
	}else{
		this->get_parameter("port", port_);
	}
	if (!this->has_parameter(("baud"))){
		RCLCPP_WARN(this->get_logger(), "Used default parameter for baud");
		baud_ = this->declare_parameter("baud", 500000);
	}else{
		this->get_parameter("baud", baud_);
	}
	if (!this->has_parameter(("scan_id"))){
		RCLCPP_WARN(this->get_logger(), "Used default parameter for scan_id");
		scan_id_ = this->declare_parameter("scan_id", 7);
	}else{
		this->get_parameter("scan_id", scan_id_);
	}
	if (!this->has_parameter(("inverted"))){
		RCLCPP_WARN(this->get_logger(), "Used default parameter for inverted");
		inverted_ = this->declare_parameter("inverted", false);
	}else{
		this->get_parameter("inverted", inverted_);
	}
	if (!this->has_parameter(("frame_id"))){
		RCLCPP_WARN(this->get_logger(), "Used default parameter for frame_id");
		frame_id_ = this->declare_parameter("frame_id", std::string("base_laser_link"));
	}else{
		this->get_parameter("frame_id", frame_id_);
	}
	if (!this->has_parameter(("scan_topic"))){
		RCLCPP_WARN(this->get_logger(), "Used default parameter for scan_topic");
		scan_topic_ = this->declare_parameter("scan_topic", std::string("scan"));
	}else{
		this->get_parameter("scan_topic", scan_topic_);
	}
	if (!this->has_parameter(("scan_duration"))){
		RCLCPP_WARN(this->get_logger(), "Used default parameter for scan_duration");
		scan_duration_ = this->declare_parameter("scan_duration", 0.025); //no info about that in SICK-docu, but 0.025 is believable and looks good in rviz
	}else{
		this->get_parameter("scan_duration", scan_duration_);
	}
	if (!this->has_parameter(("scan_cycle_time"))){
		RCLCPP_WARN(this->get_logger(), "Used default parameter for scan_cycle_time");
		scan_cycle_time_ = this->declare_parameter("scan_cycle_time", 0.040); //SICK-docu says S300 scans every 40ms
	}else{
		this->get_parameter("scan_cycle_time", scan_cycle_time_);
	}
	if (!this->has_parameter(("debug"))){
		RCLCPP_WARN(this->get_logger(), "Used default parameter for debug");
		debug_ = this->declare_parameter("debug", false);
	}else{
		this->get_parameter("debug", debug_);
	}
	if (!this->has_parameter(("communication_timeout"))){
		RCLCPP_WARN(this->get_logger(), "Used default parameter for communication_timeout");
		communication_timeout_ = this->declare_parameter("communication_timeout", 0.2);
	}else{
		this->get_parameter("communication_timeout", communication_timeout_);
	}

	// Read 'fields' param from parameter server
	auto params_interface = this->get_node_parameters_interface();
	if (!params_interface->get_parameter_overrides().empty()){
		// Get the fields numbers
		std::string param_prefix = "fields";
		std::vector<int> field_numbers;
		for (auto i : params_interface->get_parameter_overrides()){
			if (i.first.find(param_prefix) == 0){
				auto field_number = i.first.substr(param_prefix.size() + 1, 1);
				field_numbers.push_back(std::stoi(field_number));
			}
		}
		auto last = std::unique(field_numbers.begin(),field_numbers.end());
		field_numbers.erase(last, field_numbers.end());

		// If 'fields' parameter exists
		if (!field_numbers.empty()){
			// Get the parameters
			for (const int& field_number : field_numbers){
				RCLCPP_DEBUG(this->get_logger(), "Found field %d in params", field_number);

				std::string scale_param("fields." + std::to_string(field_number) + ".scale");
				if (!this->has_parameter(scale_param)){
					RCLCPP_ERROR(this->get_logger(), "Missing parameter scale");
					continue;
				}

				std::string start_angle_param("fields." + std::to_string(field_number) + ".start_angle");
				if (!this->has_parameter(start_angle_param)){
					RCLCPP_ERROR(this->get_logger(), "Missing parameter start_angle");
					continue;
				}

				std::string stop_angle_param("fields." + std::to_string(field_number) + ".stop_angle");
				if (!this->has_parameter(stop_angle_param)){
					RCLCPP_ERROR(this->get_logger(), "Missing parameter stop_angle");
					continue;
				}

				ScannerSickS300::ParamType param;
				param.dScale = get_parameter(scale_param).get_value<double>();
				param.dStartAngle = get_parameter(start_angle_param).get_value<double>();
				param.dStopAngle = get_parameter(stop_angle_param).get_value<double>();
				scanner_.setRangeField(field_number, param);

				RCLCPP_DEBUG(this->get_logger(), "params %f %f %f", param.dScale, param.dStartAngle, param.dStopAngle);
			}
		}else{
			// Setting defaults to be backwards compatible
			ScannerSickS300::ParamType param;
			param.dScale = 0.01;
			param.dStartAngle = -135.0 / 180.0 * M_PI;
			param.dStopAngle = 135.0 / 180.0 * M_PI;
			scanner_.setRangeField(1, param);
		}
	}

	// Configure the publishers
	laser_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, rclcpp::SensorDataQoS());
	in_standby_pub_ = this->create_publisher<std_msgs::msg::Bool>("scan_standby", 1);
	diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 1);
	timer_ = this->create_wall_timer(std::chrono::duration<double>(scan_cycle_time_), std::bind(&SickS3002::receiveScan, this));

	// Open the laser scanner
	bool bOpenScan = false;
	while (!bOpenScan && rclcpp::ok()){
		RCLCPP_INFO(this->get_logger(), "Opening scanner... (port:%s)", port_.c_str());
		// Try to open the scanner
		bOpenScan = this->open();
		// Check, if it is the first try to open scanner
		if (!bOpenScan){
			RCLCPP_ERROR(this->get_logger(), "...scanner not available on port %s. Will retry every second.", port_.c_str());
			this->publishError("...scanner not available on port");
			// TODO: Return failure instead of loop?
		}
		// Wait for scan to get ready if successful, or wait before retrying
		sleep(1);
	}
	RCLCPP_INFO(this->get_logger(), "...scanner opened successfully on port %s", port_.c_str());

	return rclcpp_CallReturn::SUCCESS;
}

rclcpp_CallReturn SickS3002::on_activate(const rclcpp_lifecycle::State &){
	RCLCPP_INFO(this->get_logger(), "Activating the node...");

	// Explicitly activate the lifecycle publishers
	laser_scan_pub_->on_activate();
	in_standby_pub_->on_activate();
	diag_pub_->on_activate();

	return rclcpp_CallReturn::SUCCESS;
}

rclcpp_CallReturn SickS3002::on_deactivate(const rclcpp_lifecycle::State &){
	RCLCPP_INFO(this->get_logger(), "Deactivating the node...");

	// Explicitly activate deactivate lifecycle publishers
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

	// Undeclare the parameters
	this->undeclare_parameter("port");
	this->undeclare_parameter("baud");
	this->undeclare_parameter("scan_id");
	this->undeclare_parameter("inverted");
	this->undeclare_parameter("frame_id");
	this->undeclare_parameter("scan_topic");
	this->undeclare_parameter("scan_duration");
	this->undeclare_parameter("scan_cycle_time");
	this->undeclare_parameter("debug");
	this->undeclare_parameter("communication_timeout");
	this->undeclare_parameter("fields");

	return rclcpp_CallReturn::SUCCESS;
}

rclcpp_CallReturn SickS3002::on_shutdown(const rclcpp_lifecycle::State & state){
	RCLCPP_INFO(this->get_logger(), "Shutdown the node from state %s.", state.label().c_str());

	// Release the shared pointers
	laser_scan_pub_.reset();
	in_standby_pub_.reset();
	diag_pub_.reset();

	// Undeclare the parameters
	this->undeclare_parameter("port");
	this->undeclare_parameter("baud");
	this->undeclare_parameter("scan_id");
	this->undeclare_parameter("inverted");
	this->undeclare_parameter("frame_id");
	this->undeclare_parameter("scan_topic");
	this->undeclare_parameter("scan_duration");
	this->undeclare_parameter("scan_cycle_time");
	this->undeclare_parameter("debug");
	this->undeclare_parameter("communication_timeout");
	this->undeclare_parameter("fields");

	return rclcpp_CallReturn::SUCCESS;
}

bool SickS3002::open(){
	return scanner_.open(port_.c_str(), baud_, scan_id_);
}

bool SickS3002::receiveScan(){
	std::vector<double> ranges, rangeAngles, intensities;
	unsigned int iSickTimeStamp, iSickNow;

	int result = scanner_.getScan(ranges, rangeAngles, intensities, iSickTimeStamp, iSickNow, debug_);
	static boost::posix_time::ptime pointTimeCommunicationOK = boost::posix_time::microsec_clock::local_time();

	if (result){
		if (scanner_.isInStandby()){
			publishWarn("scanner in standby");
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 30, "scanner on port %s in standby", port_.c_str());
			publishStandby(true);
		}else{
			publishStandby(false);
			publishLaserScan(ranges, rangeAngles, intensities, iSickTimeStamp, iSickNow);
		}

		pointTimeCommunicationOK = boost::posix_time::microsec_clock::local_time();
	}else{
		boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::local_time() - pointTimeCommunicationOK;

		if (diff.total_milliseconds() > static_cast<int>(1000 * communication_timeout_)){
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
	diagnostics.status[0].level = 0;
	diagnostics.status[0].name = this->get_namespace();
	diagnostics.status[0].message = "sick scanner running";
	diag_pub_->publish(diagnostics);
}

void SickS3002::publishError(std::string error){
	diagnostic_msgs::msg::DiagnosticArray diagnostics;
	diagnostics.header.stamp = this->now();
	diagnostics.status.resize(1);
	diagnostics.status[0].level = 2;
	diagnostics.status[0].name = this->get_namespace();
	diagnostics.status[0].message = error;
	diag_pub_->publish(diagnostics);
}

void SickS3002::publishWarn(std::string warn){
	diagnostic_msgs::msg::DiagnosticArray diagnostics;
	diagnostics.header.stamp = this->now();
	diagnostics.status.resize(1);
	diagnostics.status[0].level = 1;
	diagnostics.status[0].name = this->get_namespace();
	diagnostics.status[0].message = warn;
	diag_pub_->publish(diagnostics);
}