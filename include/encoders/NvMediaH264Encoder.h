// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#pragma once

#include "exceptions/NvidiaGmslDriverRosFatalException.h"
#include "framework/Checks.hpp"

#include <nvmedia_iep.h>
#include <dw/image/Image.h>

constexpr int BUFFER_SIZE = 1048576;  // 1 MiB

class NvMediaH264Encoder
{
public:
  NvMediaH264Encoder(const NvMediaSurfaceType* surfaceType, int width, int height, int framerate, int bitrate);
  virtual ~NvMediaH264Encoder();

  void feed_frame(const dwImageNvMedia* inNvMediaImage);
  bool bits_available();
  void pull_bits();
  std::array<uint8_t, BUFFER_SIZE>* get_buffer();
  [[nodiscard]] uint32_t getNumBytes_() const;

private:
  NvMediaVersion nvMediaVersion_;
  NvMediaDevice* nvMediaDevice_;
  NvMediaEncodeInitializeParamsH264 encodeInitParams_{};
  NvMediaEncodeConfigH264 encodeConfig_{};
  NvMediaEncodeConfigH264VUIParams* encodeConfigH264VuiParams_{};
  NvMediaEncodeRCParams rcParams_{};
  NvMediaEncodePicParamsH264 encodePicParams_{};
  NvMediaSurfaceType surfaceType_;
  NvMediaIEP* nvMediaIep_;
  NvMediaStatus nvMediaStatus_;

  // Buffer
  std::array<uint8_t, BUFFER_SIZE> buffer_;
  NvMediaBitstreamBuffer nvMediaBitstreamBuffer_{ buffer_.data(), 0, BUFFER_SIZE };

  uint32_t numBytesAvailable_;

  // config
  int width_;
  int height_;
  int framerate_;
  int bitrate_;

  void SetEncodeInitParams();
  void SetEncodeConfig();
  void SetEncodeConfigRCParam();
  void SetEncodePicParams();
};
