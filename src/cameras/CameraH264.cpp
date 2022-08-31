// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#include "cameras/CameraH264.h"

CameraH264::CameraH264(DriveworksApiWrapper* driveworksApiWrapper, const YAML::Node& config, std::string interface,
                       std::string link, ros::NodeHandle* nodehandle)
  : CameraBase(driveworksApiWrapper, config, interface, link, nodehandle)
{
  serializerUserData_.h264_publisher = &pub_h264_;
  serializerUserData_.info_publisher = &pub_info_;
  serializerUserData_.timestamp = &timestamp_;
  serializerUserData_.frame_id = &frame_;
  serializerUserData_.camera_info = &camera_info_;

  nh_.param<int>("h264_bitrate", bitrate_, 8000000);
  ROS_INFO_STREAM("H264 Bitrate: " << bitrate_);

  // ROS
  pub_h264_ =
      nh_.advertise<h264_image_transport_msgs::H264Packet>(config_["topic"].as<std::string>() + "/image/h264", 1);

  // Encoder
  encoder_ = std::make_unique<DriveWorksH264Serializer>(&sensorHandle_, framerate_, &serializerUserData_, bitrate_);

  // NvMedia Encoder
  //  NVM_SURF_FMT_SET_ATTR_YUV(attrs_, YUV, 422, PLANAR, UINT, 8, PL)
  NVM_SURF_FMT_SET_ATTR_YUV(attrs_, YUV, 420, SEMI_PLANAR, UINT, 8, BL)
  surfaceType_ = NvMediaSurfaceFormatGetType(attrs_, 7);
  imageConverter_ =
      std::make_unique<ImageConverter>(driveworksApiWrapper_, 1920, 1208, DW_IMAGE_NVMEDIA,
                                       DW_IMAGE_FORMAT_YUV420_UINT8_SEMIPLANAR, DW_IMAGE_MEMORY_TYPE_BLOCK);
  nvmedia_encoder_ = std::make_unique<NvMediaH264Encoder>(&surfaceType_);
}

void CameraH264::run_pipeline()
{
  poll();
  encode();
}

void CameraH264::poll()
{
  if (!get_last_frame()) {
    throw NvidiaGmslDriverRosMinorException("Unable to get frame");
  }

  CHK_DW(dwSensorCamera_getImage(&imageHandleOriginal_, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, cameraFrameHandle_));
  CHK_DW(dwImage_getTimestamp(&timestamp_, imageHandleOriginal_));
}

void CameraH264::encode()
{
  std::unique_ptr<dwImageHandle_t> converted = imageConverter_->convert(&imageHandleOriginal_);

  dwImageNvMedia* image_nvmedia_;
  CHK_DW(dwImage_getNvMedia(&image_nvmedia_, *converted));
  nvmedia_encoder_->feed_frame(image_nvmedia_);
}
