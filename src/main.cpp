// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#include "Sekonix_driver.h"
#include "framework/Log.hpp"
#include "exceptions/SekonixDriverFatalException.h"
#include <dw/core/VersionCurrent.h>
#include <ros/ros.h>
#include <unistd.h>

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
      dwLogger_initialize(getConsoleLoggerCallback(true));
      dwLogger_setLogLevel(DW_LOG_VERBOSE);
    }
  }

  ros::init(argc, argv, "sekonix_camera_ut");
  ros::NodeHandle nh("~");

  double framerate = 0;
  nh.param<double>("framerate", framerate, 30);

  ros::Rate rate(std::min(framerate, 30.0));
  std::unique_ptr<Sekonix_driver> driver;
  driver = std::make_unique<Sekonix_driver>(&nh);

  try {
    driver->setup_cameras();
  }
  catch (SekonixDriverFatalException& e) {
    ROS_FATAL_STREAM(e.what());
    ros::shutdown();
    return 1;
  }

  while (ros::ok()) {
    try {
      driver->poll_and_process();
    }
    catch (SekonixDriverFatalException& e) {
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
