cmake_minimum_required(VERSION 3.5)
project(sicks300_2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################
## Find  dependencies                         ##
################################################
## Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)

###########
## Build ##
###########
set(library_name ${PROJECT_NAME}_core)
set(executable_name ${PROJECT_NAME})

# Scanner library
add_library(scanner_serial SHARED
  src/common/ScannerSickS300.cpp
  src/common/SerialIO.cpp
)
target_include_directories(scanner_serial PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

# Main library
add_library(${library_name} SHARED
  src/sicks300.cpp
)
target_include_directories(${library_name} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${library_name}
  PUBLIC
  ${diagnostic_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${sensor_msgs_TARGETS}
  scanner_serial
  PRIVATE
  rclcpp_components::component
)

# Main executable
add_executable(${executable_name}
  src/main.cpp
)
target_link_libraries(${executable_name}
  PRIVATE
  ${library_name}
  rclcpp::rclcpp
)

rclcpp_components_register_nodes(${library_name} "sicks300_2::SickS300")

# Scan filter executable
add_executable(scan_filter
  src/scan_filter.cpp
)
target_link_libraries(scan_filter
  PRIVATE
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

#############
## Install ##
#############
install(TARGETS ${library_name} scanner_serial
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name} scan_filter
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS scripts/find_minimums.py
  DESTINATION lib/${PROJECT_NAME}
)

# ###########
# Testing  ##
# ###########
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  # add_subdirectory(test)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name} scanner_serial)
ament_export_dependencies(
  diagnostic_msgs
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  sensor_msgs
)
ament_export_targets(${PROJECT_NAME})
ament_package()