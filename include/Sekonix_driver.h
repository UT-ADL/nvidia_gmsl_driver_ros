// Created by Maxandre Ogeret.
// (c) 2021 University of Tartu - Autonomous Driving Lab.

#ifndef SEKONIX_CAMERA_UT_REWRITE_SEKONIX_DRIVER_H
#define SEKONIX_CAMERA_UT_REWRITE_SEKONIX_DRIVER_H

#include <ros/ros.h>
#include <ros/package.h>
#include "yaml-cpp/yaml.h"
#include <dw/sensors/Sensors.h>

#include <DriveworksApiWrapper.h>
#include <Camera.h>

class Sekonix_driver {
public:
  /**
   * Constructor
   * @param nodehandle
   */
  Sekonix_driver(ros::NodeHandle* nodehandle);

  /**
   * Initializes all parameters and config.
   */
  bool setup_cameras();

  bool poll_and_process();

private:
  const size_t MAX_TRIES_ = 100;
  size_t tries_ = 0;
  ros::NodeHandle nh_;
  std::string config_file_path_;
  YAML::Node config_;
  dwSALHandle_t sal_handle_ = DW_NULL_HANDLE;
  std::shared_ptr<DriveworksApiWrapper> driveworksApiWrapper_;
  size_t camera_count = 0;
  std::vector<std::shared_ptr<Camera>> camera_vector_;

//  ---
public:
  dwContextHandle_t m_context     = DW_NULL_HANDLE;
  dwSALHandle_t m_sal             = DW_NULL_HANDLE;
  dwSensorHandle_t m_cameraMaster = DW_NULL_HANDLE;
  dwImageHandle_t m_rgbaFrame[1]  = {DW_NULL_HANDLE};
  dwSensorHandle_t m_camera[1];
};

#endif // SEKONIX_CAMERA_UT_REWRITE_SEKONIX_DRIVER_H
