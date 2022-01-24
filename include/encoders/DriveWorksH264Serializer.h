// Created by Maxandre Ogeret.
// (c) 2021 University of Tartu - Autonomous Driving Lab.

#ifndef SEKONIX_CAMERA_UT_DRIVEWORKSH264SERIALIZER_H
#define SEKONIX_CAMERA_UT_DRIVEWORKSH264SERIALIZER_H

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <h264_image_transport_msgs/H264Packet.h>

#include <dw/sensors/SensorSerializer.h>

#include "framework/Checks.hpp"

/**
 * Struct passed to the serializer.
 */
struct serializer_user_data_t_
{
  ros::Publisher* h264_publisher;
  ros::Publisher* info_publisher;
  dwTime_t* timestamp;
  std::string* frame_id;
  sensor_msgs::CameraInfo* camera_info;
};

class DriveWorksH264Serializer
{
public:
  DriveWorksH264Serializer(dwSensorHandle_t* sensorHandle, int framerate, serializer_user_data_t_* user_data);
  virtual ~DriveWorksH264Serializer();
  void feed_frame(dwCameraFrameHandle_t& cameraFrameHandle);

private:
  std::string serializer_config_string_;
  dwSensorSerializerHandle_t camera_serializer_ = DW_NULL_HANDLE;

  dwSensorHandle_t* sensorHandle_;
  int framerate_;
  serializer_user_data_t_* user_data_;
};

#endif  // SEKONIX_CAMERA_UT_DRIVEWORKSH264SERIALIZER_H
