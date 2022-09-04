// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#pragma once

#include <memory>

#include <dw/sensors/SensorSerializer.h>
#include <sensor_msgs/CompressedImage.h>

#include "encoders/DriveWorksH264Serializer.h"
#include "encoders/NvMediaH264Encoder.h"
#include "cameras/CameraBase.h"
#include "tools/ImageConverter.h"

class CameraH264 : public CameraBase
{
public:
  /**
   * @brief Constructor, initializes encoder and h264 image publisher.
   * @throws NvidiaGmslDriverRosFatalException
   * @param driveworksApiWrapper
   * @param config
   * @param interface
   * @param link
   * @param nodehandle
   */
  CameraH264(DriveworksApiWrapper* driveworksApiWrapper, const YAML::Node& config, std::string interface,
             std::string link, ros::NodeHandle* nodehandle);

  /**
   * @brief Calls poll, encode. Doesn't call publish as it is part of the encoder's callback.
   */
  void run_pipeline() override;

  /**
   * @brief Polls camera for frame, extracts image and timestamp.
   * @throws NvidiaGmslDriverRosFatalException
   * @throws NvidiaGmslDriverRosMinorException
   */
  void poll() override;

  /**
   * @brief Pushes polled data to the encoder. The encoder will then call it's own callback.
   * @throws NvidiaGmslDriverRosFatalException
   */
  void encode() override;

  /**
   * @brief Publishes compressed images and camera info to ROS.
   */
  void publish() override;

  inline static const std::string ENCODER_TYPE_ = "h264";

private:
  ros::Publisher pub_compressed_;
  std::unique_ptr<DriveWorksH264Serializer> encoder_;

  int bitrate_;
  serializer_user_data_t_ serializerUserData_{};

  // new encoder todo: clean
  std::unique_ptr<NvMediaH264Encoder> nvmedia_encoder_;
  NvMediaSurfFormatAttr attrs_[7];
  NvMediaSurfaceType surfaceType_;
  std::unique_ptr<ImageConverter> imageConverter_;
  std_msgs::Header header_;
  h264_image_transport_msgs::H264Packet packet_;
};
