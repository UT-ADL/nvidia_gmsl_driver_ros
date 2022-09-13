// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#pragma once

#include <nvmedia_iep.h>
#include <dw/image/Image.h>

#include "exceptions/NvidiaGmslDriverRosFatalException.h"
#include "framework/Checks.hpp"
#include "cameras/CameraCommon.h"
#include "DriveworksApiWrapper.h"

class NvMediaH264Encoder
{
public:
  static constexpr int BUFFER_SIZE = 1048576;  // 1 MiB

  NvMediaH264Encoder(DriveworksApiWrapper* driveworksApiWrapper, int width, int height, int framerate, int bitrate);
  virtual ~NvMediaH264Encoder();

  void feed_frame(const dwImageHandle_t* input);
  bool bits_available();
  void pull_bits();
  std::array<uint8_t, BUFFER_SIZE>* get_buffer();
  [[nodiscard]] uint32_t get_num_bytes() const;

private:
  DriveworksApiWrapper* driveworksApiWrapper_;

  NvMediaVersion nvMediaVersion_;
  NvMediaDevice* nvMediaDevice_;
  NvMediaEncodeInitializeParamsH264 encodeInitParams_{};
  NvMediaEncodeConfigH264 encodeConfig_{};
  NvMediaEncodeConfigH264VUIParams* encodeConfigH264VuiParams_{};
  NvMediaEncodeRCParams rcParams_{};
  NvMediaEncodePicParamsH264 encodePicParams_{};
  NvMediaSurfaceType surfaceType_ = camera_common::get_yuv420_block_img_surface();
  NvMediaIEP* nvMediaIep_;
  NvMediaStatus nvMediaStatus_;

  dwImageHandle_t imgYuv420Bl_ = DW_NULL_HANDLE;
  dwImageNvMedia* image_nvmedia_ = nullptr;

  // Buffer
  std::array<uint8_t, BUFFER_SIZE> buffer_;
  NvMediaBitstreamBuffer nvMediaBitstreamBuffer_{ buffer_.data(), 0, BUFFER_SIZE };

  uint32_t numBytesAvailable_;

  // config
  int width_;
  int height_;
  int framerate_;
  int bitrate_;

  void set_encode_init_params();
  void set_encode_config();
  void set_encode_config_rcparam();
  void set_encode_pic_params();
};
