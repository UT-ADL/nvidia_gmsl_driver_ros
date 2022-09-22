// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#include "cameras/CameraJpg.h"

CameraJpg::CameraJpg(DriveworksApiWrapper* driveworksApiWrapper, const YAML::Node& config, std::string interface,
                     std::string link, ros::NodeHandle* nodehandle)
  : CameraBase(driveworksApiWrapper, config, interface, link, nodehandle)
{
  pub_compressed_ =
      nh_.advertise<sensor_msgs::CompressedImage>(config_["topic"].as<std::string>() + "/image/compressed", 1);

  encoder_ = std::make_unique<NvMediaJpgEncoder>(driveworksApiWrapper_, width_, height_);
}

void CameraJpg::encode()
{
  encoder_->feed_frame(transformation_needed_ ? &imgTransformed_ : &imgOutOfCamera_);
  encoder_->wait_for_bits();
  encoder_->pull_bits();
}

void CameraJpg::publish()
{
  header_.stamp = ros::Time(static_cast<double>(timestamp_) * 10e-7);
  header_.frame_id = frame_;
  img_msg_compressed_.data.assign(encoder_->get_buffer(), encoder_->get_buffer() + encoder_->get_num_bytes_available());
  img_msg_compressed_.header = header_;
  img_msg_compressed_.format = ENCODER_TYPE;
  pub_compressed_.publish(img_msg_compressed_);

  camera_info_.header = header_;
  pub_info_.publish(camera_info_);
}
