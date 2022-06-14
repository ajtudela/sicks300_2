# sicks300_2

![ROS2](https://img.shields.io/badge/ros2-galactic-purple?style=for-the-badge&logo=ros&logoColor=white)

## Overview

This package implements a driver for the Sick S300 Safety laser scanners with an interface for ROS 2. 
It provides an implementation for both, the old (1.40) and the new (2.10) protocol.
Thus, the old Sick S300 Professional CMS as well as the new Sick S300 Expert are supported.

However, it does not cover the full functionality of the protocol:
- It only handles distance measurements properly
- It only handles no or only one configured measurement range field properly
- It does not handle I/O-data or reflector data
(though it reads the reflector marker field in the distance measurements)

**Keywords:** ROS2, laser, driver, sick s300

The sicks300_2 package has been tested under [ROS2] Galactic on [Ubuntu] 20.04. This code is mainly based on [cob_sick_s300](http://wiki.ros.org/cob_sick_s300) but ported to ROS2; expect that it changes often and any fitness for a particular purpose is disclaimed.

## S300 Configuration
Here are a few notes about how to best configure the S300:
- Configure the RS422 output to 500kBaud (otherwise, the scanner only provides a lower frequency)
- Configure the scanner to Continuous Data Output
- Send data via one telegram
- Only configure distances, no I/O or reflector data (otherwise, the scanner only provides a lower frequency).
- Configuration of the measurement ranges
    - For protocol 1.40: only configure one measurement range field with the full range (-45° to 225°) with all values.
    - For protocol 2.10: do not configure a measurement range field
      (otherwise, the scanner only provides a lower frequency).
- If you want to only use certain measurement ranges, do this on the ROS side using e.g. the `laser_filters` package.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS) 2](https://docs.ros.org/en/galactic/) (middleware for robotics),
- [Boost](https://www.boost.org/) (C++ source libraries)

		rosdep install -i --from-path src --rosdistro galactic -y

#### Building

To build from source, clone the latest version from this repository into your colcon workspace and compile the package using

	cd colcon_workspace/src
	git clone https://github.com/ajtudela/sicks300_2.git
	cd ../
	rosdep install -i --from-path src --rosdistro galactic -y
	colcon build

## Usage

Run the sicks300_2 node with:

	ros2 run sicks300_2 sicks300_2

## Nodes

### sicks300_2

Driver for the Sick S300 Safety laser scanners.

#### Published Topics

* **`scan`** ([sensor_msgs/LaserScan])

	The laserscan data.

* **`scan_standby`** ([std_msgs/Bool])

	True if the scanner is in standby mode, false otherwise.

* **`/diagnostics`** ([diagnostic_msgs/DiagnosticArray])

	Diagnostic about the laser scan.

#### Parameters

* **`port`** (string, default: "/dev/ttyUSB0")

	USB port of the scanner.

* **`baud`** (int, default: 500000)

	Baudrate to communicate with the laser scanner.

* **`scan_id`** (int, default: 7)

	Identifer of the scanner.

* **`inverted`** (bool, default: false)

	Option to invert the direction of the measurements.

* **`scan_topic`** (string, default: "scan")

	The topic where the laser scan will be published.

* **`frame_id`** (string, default: "base_laser_link")

	The frame identifier of the scanner.

* **`scan_duration`** (double, default: 0.025)

	Time between laser scans.

* **`scan_cycle_time`** (double, default: 0.040))

	Cycle time of the scan. Documentation says S300 scans every 40ms.

* **`debug`** (bool, default: false)

	Option to toggle scanner debugging information.

* **`communication_timeout`** (double, default: 0.2)

	Timeout to shutdown the node.

[Ubuntu]: https://ubuntu.com/
[ROS2]: https://docs.ros.org/en/galactic/
[sensor_msgs/LaserScan]: https://docs.ros2.org/galactic/api/sensor_msgs/msg/LaserScan.html
[std_msgs/Bool]: https://docs.ros2.org/galactic/api/std_msgs/msg/Bool.html
[diagnostic_msgs/DiagnosticArray]: https://docs.ros2.org/galactic/api/diagnostic_msgs/msg/DiagnosticArray.html