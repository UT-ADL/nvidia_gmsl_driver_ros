// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#include "cameras/CameraH264.h"

CameraH264::CameraH264(DriveworksApiWrapper* driveworksApiWrapper, const YAML::Node& config, std::string interface,
                       std::string link, ros::NodeHandle* nodehandle)
  : CameraBase(driveworksApiWrapper, config, interface, link, nodehandle)
{
  nh_.param<int>("h264_bitrate", bitrate_, DEFAULT_BITRATE);
  ROS_INFO_STREAM("H264 Bitrate: " << bitrate_);

  // ROS
  pub_compressed_ =
      nh_.advertise<h264_image_transport_msgs::H264Packet>(config_["topic"].as<std::string>() + "/image/h264", 1);

  // NvMedia Encoder
  NVM_SURF_FMT_SET_ATTR_YUV(attrs_, YUV, 420, SEMI_PLANAR, UINT, 8, BL)
  surfaceType_ = NvMediaSurfaceFormatGetType(attrs_, 7);
  imageConverter_ =
      std::make_unique<ImageConverter>(driveworksApiWrapper_, 1920, 1208, DW_IMAGE_NVMEDIA,
                                       DW_IMAGE_FORMAT_YUV420_UINT8_SEMIPLANAR, DW_IMAGE_MEMORY_TYPE_BLOCK);
  nvmedia_encoder_ = std::make_unique<NvMediaH264Encoder>(&surfaceType_, width_, height_, framerate_, bitrate_);
}

void CameraH264::run_pipeline()
{
  poll();
  preprocess();
  encode();
  publish();
}

void CameraH264::encode()
{
  std::unique_ptr<dwImageHandle_t> converted = imageConverter_->convert(imageHandlePreprocessed_.get());
  CHK_DW(dwSensorCamera_returnFrame(&cameraFrameHandle_));

  dwImageNvMedia* image_nvmedia_;
  CHK_DW(dwImage_getNvMedia(&image_nvmedia_, *converted));
  nvmedia_encoder_->feed_frame(image_nvmedia_);

  dwImage_destroy(*converted);

  if (!nvmedia_encoder_->bits_available()) {
    return;
  }
  nvmedia_encoder_->pull_bits();
}

void CameraH264::publish()
{
  header_.stamp = ros::Time(static_cast<double>(timestamp_) * 10e-7);
  header_.frame_id = frame_;

  packet_.data.assign(nvmedia_encoder_->get_buffer()->data(),
                      nvmedia_encoder_->get_buffer()->data() + nvmedia_encoder_->get_num_bytes());
  packet_.header = header_;
  pub_compressed_.publish(packet_);

  camera_info_.header = header_;
  pub_info_.publish(camera_info_);
}
