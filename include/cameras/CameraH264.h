// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#pragma once

#include <sensor_msgs/CompressedImage.h>
#include <nvmedia_common_encode.h>

#include <memory>

#include "cameras/CameraBase.h"
#include "encoders/NvMediaH264Encoder.h"

/**
 * @brief Camera publishing H264 frames.
 */
class CameraH264 : public CameraBase
{
public:
  /**
   * @brief Constructor, initializes encoder and h264 image publisher.
   */
  CameraH264(DriveworksApiWrapper* driveworksApiWrapper, const YAML::Node& config, std::string interface,
             std::string link, ros::NodeHandle* nodehandle);

  /**
   * @brief Default constructor.
   */
  ~CameraH264() override = default;

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

  inline static const std::string ENCODER_TYPE = "h264";

private:
  constexpr static int DEFAULT_BITRATE = 8000000;
  constexpr static NvMediaEncodeProfile DEFAULT_PROFILE = NVMEDIA_ENCODE_PROFILE_AUTOSELECT;
  constexpr static NvMediaEncodeLevel DEFAULT_LEVEL = NVMEDIA_ENCODE_LEVEL_AUTOSELECT;

  // params
  int bitrate_;
  NvMediaEncodeProfile profile_;
  NvMediaEncodeLevel level_;

  ros::Publisher pub_compressed_;
  std_msgs::Header header_;
  sensor_msgs::CompressedImage packet_;

  std::unique_ptr<NvMediaH264Encoder> encoder_;
};
