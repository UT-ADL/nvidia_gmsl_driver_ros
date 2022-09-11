// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#include "cameras/CameraH264.h"

CameraH264::CameraH264(DriveworksApiWrapper* driveworksApiWrapper, const YAML::Node& config, std::string interface,
                       std::string link, ros::NodeHandle* nodehandle)
  : CameraBase(driveworksApiWrapper, config, interface, link, nodehandle)
{
  nh_.param<int>("h264_bitrate", bitrate_, DEFAULT_BITRATE);
  ROS_INFO_STREAM("H264 Bitrate: " << bitrate_);

  pub_compressed_ = nh_.advertise<h264_image_transport_msgs::H264Packet>(
      config_["topic"].as<std::string>() + "/image/" + ENCODER_TYPE, 1);

  encoder_ = std::make_unique<NvMediaH264Encoder>(driveworksApiWrapper_, width_, height_, framerate_, bitrate_);
}

CameraH264::~CameraH264() {}

void CameraH264::encode()
{
  encoder_->feed_frame(transformation_needed_ ? &imgTransformed_ : &imgOutOfCamera_);

  if (!encoder_->bits_available()) {
    return;
  }
  encoder_->pull_bits();
}

void CameraH264::publish()
{
  header_.stamp = ros::Time(static_cast<double>(timestamp_) * 10e-7);
  header_.frame_id = frame_;

  packet_.data.assign(encoder_->get_buffer()->data(), encoder_->get_buffer()->data() + encoder_->get_num_bytes());
  packet_.header = header_;
  pub_compressed_.publish(packet_);

  camera_info_.header = header_;
  pub_info_.publish(camera_info_);
}
