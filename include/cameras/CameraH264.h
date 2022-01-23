// Created by Maxandre Ogeret.
// (c) 2021 University of Tartu - Autonomous Driving Lab.

#ifndef SEKONIX_CAMERA_UT_CAMERAH264_H
#define SEKONIX_CAMERA_UT_CAMERAH264_H

#include "cameras/CameraBase.h"

#include <camera_info_manager/camera_info_manager.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <h264_image_transport_msgs/H264Packet.h>

#include "yaml-cpp/yaml.h"
#include <chrono>
#include <sstream>
#include <thread>

#include <dw/image/Image.h>
#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>

#include "DriveworksApiWrapper.h"
#include "framework/Checks.hpp"
#include "exceptions/SekonixDriverFatalException.h"
#include "exceptions/SekonixDriverMinorException.h"
#include "encoders/DriveWorksH264Serializer.h"

#include <dw/sensors/SensorSerializer.h>

class CameraH264 : public CameraBase
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
  CameraH264(std::shared_ptr<DriveworksApiWrapper> driveworksApiWrapper, const YAML::Node& config,
             std::string interface, std::string link, ros::NodeHandle* nodehandle);

  /**
   * @brief Polls camera for frame, extracts image, returns frame to sensor
   * @throws SekonixFatalException
   * @throws SekonixMinorException
   */
  void poll() override;

  /**
   * @brief Gets jpg data from extracted image.
   * @throws SekonixFatalException
   */
  void encode() override;

private:
  ros::Publisher pub_h264_;
  std::unique_ptr<DriveWorksH264Serializer> encoder_;

  // Serializer
  //  dwSensorSerializerHandle_t camera_serializer_ = DW_NULL_HANDLE;
  //  std::string serializer_config_string_;
  serializer_user_data_t_ serializerUserData_{};
};

#endif  // SEKONIX_CAMERA_UT_CAMERAH264_H
