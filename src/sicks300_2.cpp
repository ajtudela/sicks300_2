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

#include "sicks300_2/sicks300_2.hpp"

SickS3002::SickS3002(const std::string& name): Node(name){
	// Initialize node
	if (!this->has_parameter(("port"))){
		RCLCPP_WARN(this->get_logger(), "Used default parameter for port");
		this->declare_parameter("port");
	}
	this->get_parameter_or("port", port_, std::string("/dev/ttyUSB0"));

	if (!this->has_parameter(("baud"))){
		RCLCPP_WARN(this->get_logger(), "Used default parameter for baud");
		this->declare_parameter("baud");
	}
	this->get_parameter_or("baud", baud_, 500000);

	if (!this->has_parameter(("scan_id"))){
		RCLCPP_WARN(this->get_logger(), "Used default parameter for scan_id");
		this->declare_parameter("scan_id");
	}
	this->get_parameter_or("scan_id", scanId_, 7);

	if (!this->has_parameter(("inverted"))){
		RCLCPP_WARN(this->get_logger(), "Used default parameter for inverted");
		this->declare_parameter("inverted");
	}
	this->get_parameter_or("inverted", inverted_, false);

	if (!this->has_parameter(("frame_id"))){
		RCLCPP_WARN(this->get_logger(), "Used default parameter for frame_id");
		this->declare_parameter("frame_id");
	}
	this->get_parameter_or("frame_id", frameId_, std::string("base_laser_link"));

	if (!this->has_parameter(("scan_duration"))){
		RCLCPP_WARN(this->get_logger(), "Used default parameter for scan_duration");
		this->declare_parameter("scan_duration");
	}
	this->get_parameter_or("scan_duration", scanDuration_, 0.025); //no info about that in SICK-docu, but 0.025 is believable and looks good in rviz

	if (!this->has_parameter(("scan_cycle_time"))){
		RCLCPP_WARN(this->get_logger(), "Used default parameter for scan_cycle_time");
		this->declare_parameter("scan_cycle_time");
	}
	this->get_parameter_or("scan_cycle_time", scanCycleTime_, 0.040); //SICK-docu says S300 scans every 40ms

	if (!this->has_parameter(("debug"))){
		RCLCPP_WARN(this->get_logger(), "Used default parameter for debug");
		this->declare_parameter("debug");
	}
	this->get_parameter_or("debug", debug_, false);

	if (!this->has_parameter(("communication_timeout"))){
		RCLCPP_WARN(this->get_logger(), "Used default parameter for communication_timeout");
		this->declare_parameter("communication_timeout");
	}
	this->get_parameter_or("communication_timeout", communicationTimeout_, 0.2);

	/*try{
		//get params for each measurement
		XmlRpc::XmlRpcValue field_params;
		if(nh.getParam("fields",field_params) && field_params.getType() == XmlRpc::XmlRpcValue::TypeStruct)
		{
			for(XmlRpc::XmlRpcValue::iterator field=field_params.begin(); field!=field_params.
			end(); field++)
			{
				int field_number = boost::lexical_cast<int>(field->first);
				ROS_DEBUG("Found field %d in params", field_number);

				if(!field->second.hasMember("scale"))
				{
					ROS_ERROR("Missing parameter scale");
					continue;
				}

				if(!field->second.hasMember("start_angle"))
				{
					ROS_ERROR("Missing parameter start_angle");
					continue;
				}

				if(!field->second.hasMember("stop_angle"))
				{
					ROS_ERROR("Missing parameter stop_angle");
					continue;
				}

				ScannerSickS300::ParamType param;
				param.dScale = field->second["scale"];
				param.dStartAngle = field->second["start_angle"];
				param.dStopAngle = field->second["stop_angle"];
				scanner_.setRangeField(field_number, param);

				ROS_DEBUG("params %f %f %f", param.dScale, param.dStartAngle, param.dStopAngle);
			}
		}
		else
		{
			//ROS_WARN("No params for the Sick S300 fieldset were specified --> will using default, but it's deprecated now, please adjust parameters!!!");

			//setting defaults to be backwards compatible
			ScannerSickS300::ParamType param;
			param.dScale = 0.01;
			param.dStartAngle = -135.0/180.0*M_PI;
			param.dStopAngle = 135.0/180.0*M_PI;
			scanner_.setRangeField(1, param);
		}
	}catch(XmlRpc::XmlRpcException e){
		ROS_ERROR_STREAM("Not all params for the Sick S300 fieldset could be read: " << e.getMessage() << "! Error code: " << e.getCode());
		ROS_ERROR("Node is going to shut down.");
		exit(-1);
	}*/

	syncedSICKStamp_ = 0;
	syncedROSTime_ = this->now();
	syncedTimeReady_ = false;

	// Implementation of topics to publish
	laserScanPub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 1);
	inStandbyPub_ = this->create_publisher<std_msgs::msg::Bool>("scan_standby", 1);
	diagPub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 1);
}

SickS3002::~SickS3002(){
}

bool SickS3002::open(){
	return scanner_.open(port_.c_str(), baud_, scanId_);
}

std::string SickS3002::getPort(){
	return port_;
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

		if (diff.total_milliseconds() > static_cast<int>(1000*communicationTimeout_)){
			RCLCPP_WARN(this->get_logger(), "Communication timeout");
			return false;
		}
	}

	return true;
}

void SickS3002::publishStandby(bool inStandby){
	inStandby_.data = inStandby;
	inStandbyPub_->publish(inStandby_);
}

void SickS3002::publishLaserScan(std::vector<double> vdDistM, std::vector<double> vdAngRAD, std::vector<double> vdIntensAU, unsigned int iSickTimeStamp, unsigned int iSickNow){
	// Fill message
	int start_scan, stop_scan;
	int num_readings = vdDistM.size(); // initialize with max scan size
	start_scan = 0;
	stop_scan = vdDistM.size();

	// Sync handling: find out exact scan time by using the syncTime-syncStamp pair:
	// Timestamp: "This counter is internally incremented at each scan, i.e. every 40 ms (S300)"
	if (iSickNow != 0){
		syncedROSTime_ = this->now() - rclcpp::Duration::from_seconds(scanCycleTime_);
		syncedSICKStamp_ = iSickNow;
		syncedTimeReady_ = true;

		RCLCPP_DEBUG(this->get_logger(), "Got iSickNow, store sync-stamp: %d", syncedSICKStamp_);
	}else{
		syncedTimeReady_ = false;
	}

	// Create LaserScan message
	sensor_msgs::msg::LaserScan laserScan;
	if (syncedTimeReady_){
		double timeDiff = (int)(iSickTimeStamp - syncedSICKStamp_) * scanCycleTime_;
		laserScan.header.stamp = syncedROSTime_ + rclcpp::Duration::from_seconds(timeDiff);

		RCLCPP_DEBUG(this->get_logger(), "Time::now() - calculated sick time stamp = %f",(this->now() - laserScan.header.stamp).seconds());
	}else{
		laserScan.header.stamp = this->now();
	}

	// Fill message
	laserScan.header.frame_id = frameId_;
	laserScan.angle_increment = vdAngRAD[start_scan + 1] - vdAngRAD[start_scan];
	laserScan.range_min = 0.001;
	laserScan.range_max = 29.5; // though the specs state otherwise, the max range reported by the scanner is 29.96m
	laserScan.time_increment = (scanDuration_) / (vdDistM.size());

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
		laserScan.header.stamp = rclcpp::Time(laserScan.header.stamp) - rclcpp::Duration::from_seconds(scanDuration_); //to be consistent with the omission of the addition above
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
	laserScanPub_->publish(laserScan);

	// Diagnostics
	diagnostic_msgs::msg::DiagnosticArray diagnostics;
	diagnostics.header.stamp = this->now();
	diagnostics.status.resize(1);
	diagnostics.status[0].level = 0;
	diagnostics.status[0].name = this->get_namespace();
	diagnostics.status[0].message = "sick scanner running";
	diagPub_->publish(diagnostics);
}

void SickS3002::publishError(std::string error){
	diagnostic_msgs::msg::DiagnosticArray diagnostics;
	diagnostics.header.stamp = this->now();
	diagnostics.status.resize(1);
	diagnostics.status[0].level = 2;
	diagnostics.status[0].name = this->get_namespace();
	diagnostics.status[0].message = error;
	diagPub_->publish(diagnostics);
}

void SickS3002::publishWarn(std::string warn){
	diagnostic_msgs::msg::DiagnosticArray diagnostics;
	diagnostics.header.stamp = this->now();
	diagnostics.status.resize(1);
	diagnostics.status[0].level = 1;
	diagnostics.status[0].name = this->get_namespace();
	diagnostics.status[0].message = warn;
	diagPub_->publish(diagnostics);
}