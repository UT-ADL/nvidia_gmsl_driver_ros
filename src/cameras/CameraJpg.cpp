// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#include "cameras/CameraJpg.h"

CameraJpg::CameraJpg(DriveworksApiWrapper* driveworksApiWrapper, const YAML::Node& config, std::string interface,
                     std::string link, ros::NodeHandle* nodehandle)
  : CameraBase(driveworksApiWrapper, config, interface, link, nodehandle)
{
  // Setup image and camera properties
  CHECK_DW_ERROR_ROS(
      dwSensorCamera_getImageProperties(&imageProperties_, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, sensorHandle_))
  CHECK_DW_ERROR_ROS(dwSensorCamera_getSensorProperties(&cameraProperties_, sensorHandle_))

  // Surface type init
  NVM_SURF_FMT_SET_ATTR_YUV(attrs_, YUV, 422, PLANAR, UINT, 8, PL);
  surfaceType_ = NvMediaSurfaceFormatGetType(attrs_, 7);

  // ROS
  pub_compressed_ =
      nh_.advertise<sensor_msgs::CompressedImage>(config_["topic"].as<std::string>() + "/image/compressed", 0);

  // Encoder
  encoder_ = std::make_unique<NvMediaJpgEncoder>(&surfaceType_);
}

void CameraJpg::run_pipeline()
{
  poll();
  encode();
  publish();
}

void CameraJpg::poll()
{
  if (!get_last_frame()) {
    throw SekonixDriverMinorException("Unable to get frame");
  }

  // Get image from cameraFrameHandle
  CHECK_DW_ERROR_ROS_MINOR(
      dwSensorCamera_getImage(&imageHandleOriginal_, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, cameraFrameHandle_));

  CHECK_DW_ERROR_ROS(dwImage_getTimestamp(&timestamp_, imageHandleOriginal_));

  CHECK_DW_ERROR_ROS(dwImage_getNvMedia(&image_nvmedia_, imageHandleOriginal_));
}

void CameraJpg::encode()
{
  encoder_->feed_frame(image_nvmedia_);
  encoder_->wait_for_bits();
  CHECK_DW_ERROR_ROS(dwSensorCamera_returnFrame(&cameraFrameHandle_))
  encoder_->pull_bits();
}

void CameraJpg::publish()
{
  header_.stamp = ros::Time(static_cast<double>(timestamp_) * 10e-7);
  header_.frame_id = frame_;
  img_msg_compressed_.data.assign(encoder_->get_image(), encoder_->get_image() + encoder_->get_count_bytes());
  img_msg_compressed_.header = header_;
  img_msg_compressed_.format = "jpeg";
  pub_compressed_.publish(img_msg_compressed_);

  camera_info_.header = header_;
  pub_info_.publish(camera_info_);
}