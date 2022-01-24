// Created by Maxandre Ogeret.
// (c) 2021 University of Tartu - Autonomous Driving Lab.

#ifndef SEKONIX_CAMERA_UT_CAMERAJPG_H
#define SEKONIX_CAMERA_UT_CAMERAJPG_H

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
   * @brief Constructor, initializes the camera handles and ros params.
   * @throws SekonixFatalException
   * @param driveworksApiWrapper
   * @param config
   * @param interface
   * @param link
   * @param nodehandle
   */
  CameraJpg(std::shared_ptr<DriveworksApiWrapper> driveworksApiWrapper, const YAML::Node& config, std::string interface,
            std::string link, ros::NodeHandle* nodehandle);

  void run_pipeline() override;

  /**
   * @brief Polls camera for frame, extracts image, returns frame to sensor
   * @throws SekonixFatalException
   * @throws SekonixMinorException
   */
  void poll() override;

  /**
   * @brief Gets jpg data from extracted image.
   * @throws SekonixFatalException
   */
  void encode() override;

  void publish();

  inline static const std::string ENCODER_TYPE_ = "jpg";

private:
  std::unique_ptr<NvMediaJpgEncoder> encoder_;
  ros::Publisher pub_compressed_;
  std_msgs::Header header_;
  sensor_msgs::CompressedImage img_msg_compressed_;

  dwImageProperties imageProperties_;
  dwCameraProperties cameraProperties_;
  dwImageNvMedia* image_nvmedia_;

  NvMediaSurfFormatAttr attrs_[7];
  NvMediaSurfaceType surfaceType_;
};

#endif  // SEKONIX_CAMERA_UT_CAMERAJPG_H
