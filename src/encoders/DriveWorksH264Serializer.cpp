// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#include "encoders/DriveWorksH264Serializer.h"

DriveWorksH264Serializer::DriveWorksH264Serializer(dwSensorHandle_t* sensorHandle, int framerate,
                                                   serializer_user_data_t_* user_data, int bitrate)
  : sensorHandle_(sensorHandle), framerate_(framerate), user_data_(user_data)
{
  serializer_config_string_ += "format=h264";
  serializer_config_string_ += ",bitrate=" + std::to_string(bitrate);
  serializer_config_string_ += ",framerate=" + std::to_string(framerate_);
  serializer_config_string_ += ",type=user";

  dwSerializerParams serializerParams;

  serializerParams.parameters = serializer_config_string_.c_str();
  serializerParams.userData = user_data_;

  /**
   * onData is the callback that will be called once h264 data is ready.
   * It prepares the messages from h264 data and the data passed by userData.
   * Publishes H264Packet & camera_info msgs via the publishers passed by userData.
   * @param data The h264 bytes
   * @param size size in bytes of data
   * @param userData Pointer to a serializer_user_data_t_
   */
  serializerParams.onData = [](const uint8_t* data, size_t size, void* userData) {
    auto* userDataCast = static_cast<serializer_user_data_t_*>(userData);
    auto stamp = ros::Time(static_cast<double>(*userDataCast->timestamp) * 10e-7);

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
