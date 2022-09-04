// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#pragma once

#include <camera_info_manager/camera_info_manager.h>
#include <ros/package.h>
#include <sensor_msgs/CompressedImage.h>

#include <yaml-cpp/yaml.h>
#include <chrono>
#include <thread>
#include <memory>

#include <dw/sensors/camera/Camera.h>

#include "DriveworksApiWrapper.h"
#include "framework/Checks.hpp"

/**
 * @brief Base class for cameras.
 */
class CameraBase
{
public:
  /**
   * @brief Constructor, initializes the camera handles and ros variables.
   * @throws NvidiaGmslDriverRosFatalException
   * @param driveworksApiWrapper
   * @param config
   * @param interface
   * @param link
   * @param nodehandle
   */
  CameraBase(DriveworksApiWrapper* driveworksApiWrapper, const YAML::Node& config, std::string interface,
             std::string link, ros::NodeHandle* nodehandle);

  /**
   * @brief Destructor, releases camera handles
   */
  virtual ~CameraBase();

  /**
   * @brief Starts the sensor
   * @throws NvidiaGmslDriverRosFatalException
   */
  void start();

  /**
   * @brief Has to be overridden. Executes once all the steps that the camera implements.
   */
  virtual void run_pipeline() = 0;

  /**
   * @brief Has to be overridden. Polls camera for a frame + eventually extra processing.
   */
  virtual void poll() = 0;

  /**
   * @brief Has to be overridden. Pushes and pulls polled data to the encoder.
   */
  virtual void encode() = 0;

  virtual void publish() = 0;

  /**
   * @brief Polls the camera until the buffer is empty. ensuring the frame is the most recent one.
   */
  bool get_last_frame();

protected:
  static constexpr int DEFAULT_FRAMERATE_ = 30;
  static constexpr int DEFAULT_WIDTH_ = 1920;
  static constexpr int DEFAULT_HEIGHT_ = 1208;

  int framerate_;
  int width_;
  int height_;
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
  std::string calibDirPath_;
  std::string params_;
  std::string interface_;
  std::string link_;
  std::string frame_;
  std::ostringstream cam_info_file_;
};
