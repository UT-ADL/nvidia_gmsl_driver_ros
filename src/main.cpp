// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#include "Driver.h"
#include "log.hpp"
#include <dw/core/VersionCurrent.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  if (!(DW_VERSION.major == 3 && DW_VERSION.minor == 5)) {
    ROS_FATAL("This driver requires Driveworks 3.5 !");
    return 1;
  }

  std::vector<std::string> args_out;
  ros::removeROSArgs(argc, argv, args_out);
  for (auto arg : args_out) {
    std::cout << arg << "\n";
    if (arg == "--verbose") {
      // initialize loggers
      ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
      dwLogger_initialize(colorLogger());
      dwLogger_setLogLevel(DW_LOG_VERBOSE);
    }
  }

  ros::init(argc, argv, "nvidia_gmsl_driver_ros");
  ros::NodeHandle nh("~");

  double framerate = 0;
  nh.param<double>("framerate", framerate, 30);

  ros::Rate rate(std::min(framerate, 30.0));
  std::unique_ptr<Driver> driver;
  driver = std::make_unique<Driver>(&nh);

  try {
    driver->setup_cameras();
  }
  catch (NvidiaGmslDriverRosFatalException& e) {
    ROS_FATAL_STREAM(e.what());
    ros::shutdown();
    return 1;
  }

  while (ros::ok()) {
    try {
      driver->run();
    }
    catch (NvidiaGmslDriverRosFatalException& e) {
      ROS_FATAL_STREAM(e.what());
      ros::shutdown();
      return 1;
    }

    ROS_INFO_ONCE("Driver started !");
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
