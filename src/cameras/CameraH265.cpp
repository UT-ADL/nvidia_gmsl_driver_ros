// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#include "cameras/CameraH265.h"

CameraH265::CameraH265(DriveworksApiWrapper* driveworksApiWrapper, const YAML::Node& config, std::string interface,
                       std::string link, ros::NodeHandle* nodehandle)
  : CameraBase(driveworksApiWrapper, config, interface, link, nodehandle)
{
  nh_.param<int>("bitrate", bitrate_, DEFAULT_BITRATE);
  ROS_INFO_STREAM("H265 Bitrate: " << bitrate_);

  pub_compressed_ =
      nh_.advertise<sensor_msgs::CompressedImage>(config_["topic"].as<std::string>() + "/image/" + ENCODER_TYPE, 1);

  encoder_ = std::make_unique<NvMediaH265Encoder>(driveworksApiWrapper_, width_, height_, framerate_, bitrate_);
}

CameraH265::~CameraH265() {}

void CameraH265::encode()
{
  encoder_->feed_frame(transformation_needed_ ? &imgTransformed_ : &imgOutOfCamera_);

  if (!encoder_->bits_available()) {
    return;
  }
  encoder_->pull_bits();
}

void CameraH265::publish()
{
  header_.stamp = ros::Time().fromNSec(timestamp_ * 1e3);
  header_.frame_id = frame_;

  packet_.data.assign(encoder_->get_buffer(), encoder_->get_buffer() + encoder_->get_num_bytes_available());
  packet_.header = header_;
  packet_.format = ENCODER_TYPE;
  pub_compressed_.publish(packet_);

  camera_info_.header = header_;
  pub_info_.publish(camera_info_);
}
