// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#pragma once

#include <sensor_msgs/CompressedImage.h>

#include <memory>

#include "cameras/CameraBase.h"
#include "encoders/NvMediaVp9Encoder.h"

class CameraVp9 : public CameraBase
{
public:
  /**
   * @brief Constructor, initializes encoder and vp9 image publisher.
   * @throws NvidiaGmslDriverRosFatalException
   * @param driveworksApiWrapper
   * @param config
   * @param interface
   * @param link
   * @param nodehandle
   */
  CameraVp9(DriveworksApiWrapper* driveworksApiWrapper, const YAML::Node& config, std::string interface,
            std::string link, ros::NodeHandle* nodehandle);

  /**
   * @brief Pushes polled data to the encoder. The encoder will then call it's own callback.
   * @attention Prerequisite : preprocess()
   * @throws NvidiaGmslDriverRosFatalException
   */
  void encode() override;

  /**
   * @brief Publishes compressed images and camera info to ROS.
   * @attention Prerequisite : encode()
   */
  void publish() override;

  inline static const std::string ENCODER_TYPE = "vp9";

private:
  constexpr static int DEFAULT_BITRATE = 8000000;

  // params
  int bitrate_;

  ros::Publisher pub_compressed_;
  std_msgs::Header header_;
  sensor_msgs::CompressedImage packet_;

  std::unique_ptr<NvMediaVp9Encoder> encoder_;
};
