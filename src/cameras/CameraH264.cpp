// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#include "cameras/CameraH264.h"

CameraH264::CameraH264(std::shared_ptr<DriveworksApiWrapper> driveworksApiWrapper, const YAML::Node& config,
                       std::string interface, std::string link, ros::NodeHandle* nodehandle)
  : CameraBase(driveworksApiWrapper, config, interface, link, nodehandle)
{
  serializerUserData_.h264_publisher = &pub_h264_;
  serializerUserData_.info_publisher = &pub_info_;
  serializerUserData_.timestamp = &timestamp_;
  serializerUserData_.frame_id = &frame_;
  serializerUserData_.camera_info = &camera_info_;

  // ROS
  pub_h264_ =
      nh_.advertise<h264_image_transport_msgs::H264Packet>(config_["topic"].as<std::string>() + "/image/h264", 0);

  // Encoder
  encoder_ = std::make_unique<DriveWorksH264Serializer>(&sensorHandle_, framerate_, &serializerUserData_);
}

void CameraH264::run_pipeline()
{
  poll();
  encode();
}

void CameraH264::poll()
{
  if (!get_last_frame()) {
    throw SekonixDriverMinorException("Unable to get frame");
  }

  CHECK_DW_ERROR_ROS(
      dwSensorCamera_getImage(&imageHandleOriginal_, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, cameraFrameHandle_))
  CHECK_DW_ERROR_ROS(dwImage_getTimestamp(&timestamp_, imageHandleOriginal_))
}

void CameraH264::encode()
{
  encoder_->feed_frame(cameraFrameHandle_);
  CHECK_DW_ERROR_ROS(dwSensorCamera_returnFrame(&cameraFrameHandle_))
}