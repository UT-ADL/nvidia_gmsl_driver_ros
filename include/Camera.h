// Created by Maxandre Ogeret.
// (c) 2021 University of Tartu - Autonomous Driving Lab.

#ifndef SEKONIX_CAMERA_UT_CAMERA_H
#define SEKONIX_CAMERA_UT_CAMERA_H

#include <camera_info_manager/camera_info_manager.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include "sekonix_camera_ut/H264Packet.h"

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

#include <sstream>
#include <dw/sensors/SensorSerializer.h>

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
  Camera(std::shared_ptr<DriveworksApiWrapper> driveworksApiWrapper, const YAML::Node& config, std::string interface,
         std::string link, ros::NodeHandle* nodehandle);

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

  void start_serializer();

private:
  struct serializer_user_data_t_
  {
    ros::Publisher* publisher;
    dwTime_t* timestamp;
    std::string* frame_id;
  };

  int framerate_;
  std::shared_ptr<DriveworksApiWrapper> driveworksApiWrapper_;

  dwSensorHandle_t sensorHandle_ = DW_NULL_HANDLE;
  dwCameraFrameHandle_t cameraFrameHandle_;
  dwImageHandle_t imageHandleOriginal_;
  dwSensorParams sensorParams_;
  dwStatus status_;
  dwTime_t timestamp_;

  ros::NodeHandle nh_;
  ros::Publisher pub_h264;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

  YAML::Node config_;
  std::string calibDirPath_ = "";
  std::string params_ = "";
  std::string interface_ = "";
  std::string link_ = "";
  std::string frame_;
  std::ostringstream cam_info_file_;

  // Serializer
  dwSensorSerializerHandle_t camera_serializer_ = DW_NULL_HANDLE;
  std::string serializer_config_string_;
  serializer_user_data_t_ serializerUserData_{};
};

#endif  // SEKONIX_CAMERA_UT_CAMERA_H
