^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sicks300_2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.2 (20-12-2024)
------------------
* Improve formating and linting.
* Remove nav2_util dependency.
* CMakelists.txt use modern idioms.
* Add more restricted compiled options.

1.3.0 (20-06-2023)
------------------
* Added scan_filter node.
* Added logging info in launch file.
* Added scan delay parameter.

1.2.3 (02-05-2023)
------------------
* Update on_activate and on_deactivate methods of the publishers: https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html#rclcpp-lifecycle

1.2.2 (28-11-2022)
------------------
* Add LICENSE file.

1.2.1 (24-11-2022)
------------------
* Replace diagnostic messages with enums.
* Replace declare_parameter_if_not_declared with the one inside nav2_util.

1.2.0 (27-10-2022)
------------------
* Update find_minimums script with QoS.
* Remove boost dependencies.

1.1.1 (26-10-2022)
------------------
* Update launch file with arguments.
* Rename folder from config to params.
* Update publishers QoS.

1.1.0 (11-10-2022)
------------------
* Update parameters declarations.
* Check if parameters have been declared.
* Fix timer when transition from states.
* Rename dummy_launch.py to scan_with_filter.py.
* Remove undeclare the parameters.

1.0.0 (25-07-2022)
-------------------
* From cob_driver version 0.7.12.
* Convert to ROS2.
* Add github workflow.
* Added QoS.
* Convert node into a lifecycle node.
* Added laser_filters.
* Added script to find minimum. Thanks to Manolo Fernandez Carmona.
* Contributors: Alberto Tudela.