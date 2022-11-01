// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#include <dw/core/VersionCurrent.h>
#include <ros/ros.h>

#include "Driver.h"
#include "framework/log.hpp"

int main(int argc, char** argv)
{
  static_assert(DW_VERSION.major == 3 && DW_VERSION.minor == 5);

  ros::init(argc, argv, "nvidia_gmsl_driver_ros");
  ros::NodeHandle nh("~");

  if (nh.param<bool>("verbose", false)) {
    ROS_INFO_STREAM("Running in verbose mode.");
    // initialize loggers
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::console::notifyLoggerLevelsChanged();
    dwLogger_initialize(ros_log_wrapper());
    dwLogger_setLogLevel(DW_LOG_VERBOSE);
  }

  double framerate = nh.param<double>("framerate", 30);

  ros::Rate rate(std::min(framerate, 30.0));
  std::unique_ptr<Driver> driver;
  driver = std::make_unique<Driver>(&nh);

  try {
    driver->setup_cameras();
  } catch (NvidiaGmslDriverRosFatalException const& e) {
    ROS_FATAL_STREAM(e.what());
    ros::shutdown();
    return 1;
  }

  while (ros::ok()) {
    try {
      driver->run();
    } catch (NvidiaGmslDriverRosFatalException const& e) {
      ROS_FATAL_STREAM(e.what());
      ros::shutdown();
      return 1;
    }

    ROS_INFO_ONCE("Driver started !");
    rate.sleep();
  }
  return 0;
}
