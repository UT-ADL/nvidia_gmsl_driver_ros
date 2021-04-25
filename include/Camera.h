// Created by Maxandre Ogeret.
// (c) 2021 University of Tartu - Autonomous Driving Lab.

#ifndef SEKONIX_CAMERA_UT_CAMERA_H
#define SEKONIX_CAMERA_UT_CAMERA_H

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>

#include "yaml-cpp/yaml.h"
#include <chrono>
#include <thread>
#include <sstream>

#include <dw/sensors/Sensors.h>
#include <dw/image/Image.h>
#include <dw/sensors/camera/Camera.h>
#include <nvmedia_ijpe.h>

#include "DriveworksApiWrapper.h"
#include "framework/Checks.hpp"

class Camera {
public:
  /**
   * @brief Constructor, initializes the camera handles and ros params.
   * @param driveworksApiWrapper
   * @param config
   * @param interface
   * @param link
   * @param nodehandle
   */
  Camera(std::shared_ptr<DriveworksApiWrapper> driveworksApiWrapper, const YAML::Node config, const std::string interface, const std::string link, ros::NodeHandle *nodehandle);

  /**
   * @brief Destructor, releases camera handles
   */
  virtual ~Camera();

  /**
   * @brief Starts the sensor
   */
  bool start();

  /**
   * @brief Polls camera for frame, gets jpg data then returns frame to sensor
   */
  bool poll();

  /**
   * @brief Publishes the current jpg data and camera info
   */
  void publish();

  /**
   * @brief Destroys image handle. Allows
   */
  void clean();

private:
  dwSensorHandle_t sensorHandle_ = DW_NULL_HANDLE;
  dwCameraFrameHandle_t cameraFrameHandle_;
  dwImageHandle_t imageHandle_;
  dwImageHandle_t imageHandleOriginal_;
  dwImageProperties imageProperties_;
  dwCameraProperties cameraProperties_;
  NvMediaDevice *nvmediaDevice_ = nullptr;
  NvMediaIJPE *nvMediaIjpe_ = nullptr;
  uint32_t countByteJpeg_;
  std::unique_ptr<uint8_t[]> jpegImage_;
  const uint32_t maxJpegBytes_ = 3 * 1290 * 1208;
  NvMediaSurfFormatAttr attrs_[7];
  NvMediaSurfaceType surfaceType_;

  ros::NodeHandle nh_;
  ros::Publisher pub_compressed;
  ros::Publisher pub_info;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

  std::shared_ptr<DriveworksApiWrapper> driveworksApiWrapper_;
  dwSensorParams sensorParams_;
  dwStatus status_;
  NvMediaStatus nvMediaStatus_;
  YAML::Node config_;
  std::string calibDirPath_ = "";
  std::string params_ = "";
  std::string interface_ = "";
  std::string link_ = "";
  dwTime_t timestamp_;
  ros::Time imageStamp_;
  std::ostringstream frame_;
  std::ostringstream cam_info_file_;
  sensor_msgs::CameraInfo camera_info_;
};

#endif // SEKONIX_CAMERA_UT_CAMERA_H
