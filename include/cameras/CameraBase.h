// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#pragma once

#include <camera_info_manager/camera_info_manager.h>
#include <dw/sensors/camera/Camera.h>
#include <ros/package.h>
#include <sensor_msgs/CompressedImage.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <memory>
#include <thread>

#include "DriveworksApiWrapper.h"
#include "cameras/CameraCommon.h"
#include "framework/Checks.hpp"
#include "processors/ImageTransformer.h"

/**
 * @brief Base class for cameras.
 */
class CameraBase
{
public:
  /**
   * @brief Constructor, initializes ROS parameters, camera handles and properties, camera calibration, image
   * transformer.
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
   * @brief Executes once all the steps that the camera implements.
   */
  void run_pipeline();

  /**
   * @brief Polls the camera until the buffer is empty. ensuring the frame is the most recent one.
   * @throws NvidiaGmslDriverRosMinorException
   */
  void poll();

  /**
   * @brief Run any required preprocessing on the polled image. E.g. Image Transformer.
   * @attention Prerequisite : poll()
   */
  void preprocess();

  /**
   * @brief Has to be overridden. Pushes the polled data (preprocessed or not) to the encoder, then fetches the data.
   * @attention Prerequisite : preprocess()
   */
  virtual void encode() = 0;

  /**
   * @brief Has to be overridden. Publishes to ROS the data pulled in the encode() function.
   * @attention Prerequisite : encode()
   */
  virtual void publish() = 0;

protected:
  bool transformation_needed_ = false;
  int framerate_;
  int width_;
  int height_;
  DriveworksApiWrapper* driveworksApiWrapper_;
  std::unique_ptr<ImageTransformer> imageTransformer_;

  dwSensorHandle_t sensorHandle_ = DW_NULL_HANDLE;
  dwCameraFrameHandle_t cameraFrameHandle_;
  dwSensorParams sensorParams_;
  dwStatus status_;
  dwTime_t timestamp_;
  dwImageProperties cameraImgProps_;
  dwCameraProperties cameraProperties_;
  dwImageHandle_t imgOutOfCamera_ = DW_NULL_HANDLE;
  dwImageHandle_t imgTransformed_ = DW_NULL_HANDLE;

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
