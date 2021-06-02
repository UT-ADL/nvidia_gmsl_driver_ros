// Created by Maxandre Ogeret.
// (c) 2021 University of Tartu - Autonomous Driving Lab.

#ifndef SEKONIX_CAMERA_UT_CAMERA_H
#define SEKONIX_CAMERA_UT_CAMERA_H

#include <camera_info_manager/camera_info_manager.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>

#include "yaml-cpp/yaml.h"
#include <chrono>
#include <sstream>
#include <thread>

#include <dw/image/Image.h>
#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>
#include <nvmedia_ijpe.h>

#include "DriveworksApiWrapper.h"
#include "framework/Checks.hpp"
#include "exceptions/SekonixDriverFatalException.h"
#include "exceptions/SekonixDriverMinorException.h"
#include "encoders/NvMediaJPGEncoder.h"

class Camera
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
  Camera(std::shared_ptr<DriveworksApiWrapper> driveworksApiWrapper, const YAML::Node config,
         const std::string interface, const std::string link, ros::NodeHandle* nodehandle);

  /**
   * @brief Destructor, releases camera handles
   */
  virtual ~Camera();

  /**
   * @brief Starts the sensor
   * @throws SekonixFatalException
   */
  void start();

  /**
   * @brief Polls camera for frame, extracts image, returns frame to sensor
   * @throws SekonixFatalException
   * @throws SekonixMinorException
   */
  void poll();

  /**
   * @brief Gets jpg data from extracted image.
   * @throws SekonixFatalException
   */
  void encode();

  /**
   * @brief Polls the camera until the buffer is empty. ensuring the frame is the most recent one.
   */
  bool get_last_frame();

  /**
   * @brief Publishes the current jpg data and camera info
   */
  void publish();

  /**
   * @brief Destroys image handle. Allows
   */
  void clean();

private:
  std::shared_ptr<DriveworksApiWrapper> driveworksApiWrapper_;
  std::unique_ptr<NvMediaJPGEncoder> nvMediaJPGEncoder_;

  dwSensorHandle_t sensorHandle_ = DW_NULL_HANDLE;
  dwCameraFrameHandle_t cameraFrameHandle_;
  dwImageHandle_t imageHandle_;
  dwImageHandle_t imageHandleOriginal_;
  dwImageProperties imageProperties_;
  dwCameraProperties cameraProperties_;
  dwSensorParams sensorParams_;
  dwStatus status_;
  dwTime_t timestamp_;
  dwImageNvMedia* image_nvmedia_;

  NvMediaSurfFormatAttr attrs_[7];
  NvMediaSurfaceType surfaceType_;
  NvMediaStatus nvMediaStatus_;

  ros::NodeHandle nh_;
  ros::Publisher pub_compressed;
  ros::Publisher pub_info;
  ros::Time imageStamp_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::CameraInfo camera_info_;
  std_msgs::Header header_;
  sensor_msgs::CompressedImage img_msg_compressed_;

  YAML::Node config_;
  std::string calibDirPath_ = "";
  std::string params_ = "";
  std::string interface_ = "";
  std::string link_ = "";
  std::ostringstream frame_;
  std::ostringstream cam_info_file_;
};

#endif  // SEKONIX_CAMERA_UT_CAMERA_H
