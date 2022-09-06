// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#pragma once

#include "cameras/CameraBase.h"

#include <memory>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Header.h>
#include <condition_variable>

#include "DriveworksApiWrapper.h"
#include "encoders/NvMediaJpgEncoder.h"

class CameraJpg : public CameraBase
{
public:
  /**
   * @brief Constructor, initializes encoder and jpg image publisher.
   * @throws NvidiaGmslDriverRosFatalException
   * @param driveworksApiWrapper
   * @param config
   * @param interface
   * @param link
   * @param nodehandle
   */
  CameraJpg(DriveworksApiWrapper* driveworksApiWrapper, const YAML::Node& config, std::string interface,
            std::string link, ros::NodeHandle* nodehandle);

  /**
   * @brief Calls poll, encode, publish
   */
  void run_pipeline() override;

  /**
   * @brief Pushes polled data to the encoder and pulls jpg bytes.
   * @throws NvidiaGmslDriverRosFatalException
   */
  void encode() override;

  /**
   * @brief Publishes compressed images and camera info to ROS.
   * @attention Prerequisite : encode()
   */
  void publish() override;

  inline static const std::string ENCODER_TYPE = "jpg";

private:
  std::unique_ptr<NvMediaJpgEncoder> encoder_;
  ros::Publisher pub_compressed_;
  std_msgs::Header header_;
  sensor_msgs::CompressedImage img_msg_compressed_;

  dwImageNvMedia* image_nvmedia_;

  NvMediaSurfFormatAttr attrs_[7];
  NvMediaSurfaceType surfaceType_;
};
