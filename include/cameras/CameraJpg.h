// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#pragma once

#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Header.h>

#include <condition_variable>
#include <memory>

#include "DriveworksApiWrapper.h"
#include "cameras/CameraBase.h"
#include "encoders/NvMediaJpgEncoder.h"

/**
 * @brief Camera publishing jpg images.
 */
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
   * @brief Default constructor.
   */
  ~CameraJpg() override = default;

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
};
