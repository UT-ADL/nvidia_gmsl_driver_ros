#ifndef SEKONIX_CAMERA_GMSL_H
#define SEKONIX_CAMERA_GMSL_H

#include <ros/ros.h>
#include "DriveWorksApi.hpp"
#include "DeviceArguments.hpp"
#include <memory>
#include "PrintEventHandler.h"

class SekonixCamera {
public:
  explicit SekonixCamera(ros::NodeHandle &nh_in, PrintEventHandler::Ptr print_event_handler);

  void Shutdown();

private:
  ros::NodeHandle &nh_;
  PrintEventHandler::Ptr print_event_handler_;
  std::unique_ptr<DriveWorks::DriveWorksApi> driveworks_api_;
  std::string name_pretty_;
};


#endif //SEKONIX_CAMERA_GMSL_H
