//
// Created by Maxandre Ogeret.
//

#ifndef SEKONIX_CAMERA_UT_NVMEDIAH264ENCODER_H
#define SEKONIX_CAMERA_UT_NVMEDIAH264ENCODER_H

#include <nvmedia_iep.h>

#include "framework/Checks.hpp"
#include "exceptions/SekonixDriverFatalException.h"

class NvMediaH264Encoder
{
public:
  NvMediaH264Encoder(NvMediaSurfaceType surfaceType);
  virtual ~NvMediaH264Encoder();
  void feed_frame(NvMediaImage* inNvMediaImage);
  bool bits_available();
  void get_bits();

private:
  NvMediaStatus status_;
  NvMediaRect* rect_;
  NvMediaVersion* nvMediaVersion_;
  NvMediaDevice* nvMediaDevice_;
  NvMediaEncodeInitializeParamsH264 initParams_{};
  NvMediaEncodePicParamsH264  picParams_{};
  NvMediaEncodeConfigH264 config_{};
  NvMediaSurfaceType surfaceType_;
  NvMediaIEP* encoder_;
  uint32_t numBytesAvailable_;
  NvMediaBitstreamBuffer buffer_;
};

#endif  // SEKONIX_CAMERA_UT_NVMEDIAH264ENCODER_H
