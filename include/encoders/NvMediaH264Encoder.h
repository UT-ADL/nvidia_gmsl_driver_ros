// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#pragma once

#include "exceptions/NvidiaGmslDriverRosFatalException.h"
#include "framework/Checks.hpp"

#include <nvmedia_iep.h>
#include <dw/image/Image.h>

class NvMediaH264Encoder
{
public:
  NvMediaH264Encoder(const NvMediaSurfaceType* surfaceType);
  virtual ~NvMediaH264Encoder();

  void feed_frame(dwImageNvMedia* inNvMediaImage);

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

  void SetEncodeInitParams();
  void SetEncodeConfig();
  void SetEncodeConfigRCParam();
  void SetEncodePicParams();
};
