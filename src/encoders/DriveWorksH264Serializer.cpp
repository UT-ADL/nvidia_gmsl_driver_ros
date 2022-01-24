// Created by Maxandre Ogeret.
// (c) 2021 University of Tartu - Autonomous Driving Lab.

#include "encoders/DriveWorksH264Serializer.h"
DriveWorksH264Serializer::DriveWorksH264Serializer(dwSensorHandle_t* sensorHandle, int framerate,
                                                   serializer_user_data_t_* user_data)
  : sensorHandle_(sensorHandle), framerate_(framerate), user_data_(user_data)
{
  serializer_config_string_ =
      "format=h264"
      ",bitrate=8000000"
      ",type=user";
  serializer_config_string_ += ",framerate=" + std::to_string(framerate_);

  dwSerializerParams serializerParams;

  serializerParams.parameters = serializer_config_string_.c_str();
  serializerParams.userData = user_data_;

  serializerParams.onData = [](const uint8_t* data, size_t size, void* userData) -> void {
    auto* userDataCast = static_cast<serializer_user_data_t_*>(userData);
    auto stamp = ros::Time((static_cast<double>(*userDataCast->timestamp) * 10e-7));

    h264_image_transport_msgs::H264Packet msg;
    msg.data.assign(data, data + size);
    msg.header.stamp = stamp;
    msg.header.frame_id = *userDataCast->frame_id;
    userDataCast->h264_publisher->publish(msg);

    userDataCast->camera_info->header.stamp = stamp;
    userDataCast->camera_info->header.frame_id = *userDataCast->frame_id;
    userDataCast->info_publisher->publish(*userDataCast->camera_info);
  };

  CHECK_DW_ERROR_ROS(dwSensorSerializer_initialize(&camera_serializer_, &serializerParams, *sensorHandle_))
  CHECK_DW_ERROR_ROS(dwSensorSerializer_start(camera_serializer_))
  ROS_DEBUG("INITIALIZED SERIALIZER");
}

DriveWorksH264Serializer::~DriveWorksH264Serializer()
{
  if (camera_serializer_) {
    dwSensorSerializer_stop(camera_serializer_);
    dwSensorSerializer_release(camera_serializer_);
    ROS_DEBUG("RELEASED SERIALIZER");
  }
}

void DriveWorksH264Serializer::feed_frame(dwCameraFrameHandle_t& cameraFrameHandle)
{
  CHECK_DW_ERROR_ROS(dwSensorSerializer_serializeCameraFrameAsync(cameraFrameHandle, camera_serializer_))
}
