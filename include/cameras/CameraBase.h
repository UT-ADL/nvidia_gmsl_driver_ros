// Created by Maxandre Ogeret.
// (c) 2021 University of Tartu - Autonomous Driving Lab.

#ifndef SEKONIX_CAMERA_UT_CAMERA_BASE_H
#define SEKONIX_CAMERA_UT_CAMERA_BASE_H

#include <camera_info_manager/camera_info_manager.h>
#include <ros/package.h>

#include <yaml-cpp/yaml.h>
#include <chrono>
#include <thread>
#include <memory>

#include <dw/sensors/camera/Camera.h>

#include "DriveworksApiWrapper.h"
#include "framework/Checks.hpp"

/**
 * Base class for cameras.
 */
class CameraBase
{
public:
  /**
   * @brief Constructor, initializes the camera handles and ros params.
   * @throws SekonixFatalException
   * @param driveworksApiWrapper
   * @param config
   * @param interface
   * @param link
   * @param nodehandle
   */
  CameraBase(std::shared_ptr<DriveworksApiWrapper> driveworksApiWrapper, const YAML::Node& config,
             std::string interface, std::string link, ros::NodeHandle* nodehandle);

  /**
   * @brief Destructor, releases camera handles
   */
  virtual ~CameraBase();

  /**
   * @brief Starts the sensor
   * @throws SekonixFatalException
   */
  void start();

  virtual void run_pipeline() = 0;

  /**
   * @brief Polls camera for frame, extracts image, returns frame to sensor
   */
  virtual void poll() = 0;

  /**
   * @brief Gets jpg data from extracted image.
   */
  virtual void encode() = 0;

  /**
   * @brief Polls the camera until the buffer is empty. ensuring the frame is the most recent one.
   */
  bool get_last_frame();

protected:
  int framerate_;
  std::shared_ptr<DriveworksApiWrapper> driveworksApiWrapper_;

  dwSensorHandle_t sensorHandle_ = DW_NULL_HANDLE;
  dwCameraFrameHandle_t cameraFrameHandle_;
  dwImageHandle_t imageHandleOriginal_;
  dwSensorParams sensorParams_;
  dwStatus status_;
  dwTime_t timestamp_;

  ros::NodeHandle nh_;
  ros::Publisher pub_info_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::CameraInfo camera_info_;

  YAML::Node config_;
  std::string calibDirPath_ = "";
  std::string params_ = "";
  std::string interface_ = "";
  std::string link_ = "";
  std::string frame_;
  std::ostringstream cam_info_file_;
};

#endif  // SEKONIX_CAMERA_UT_CAMERA_BASE_H
