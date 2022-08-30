// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#include "encoders/NvMediaH264Encoder.h"

NvMediaH264Encoder::NvMediaH264Encoder(const NvMediaSurfaceType* surfaceType) : surfaceType_(*surfaceType)
{
  NvMediaIEPGetVersion(&nvMediaVersion_);
  nvMediaDevice_ = NvMediaDeviceCreate();

  if (!nvMediaDevice_) {
    throw NvidiaGmslDriverRosFatalException("Unable to create NvMedia device.");
  }

  SetEncodeInitParams();

  nvMediaIep_ = NvMediaIEPCreate(nvMediaDevice_, NVMEDIA_IMAGE_ENCODE_H264, &encodeInitParams_, surfaceType_, '\0', 16,
                                 NVMEDIA_ENCODER_INSTANCE_0);

  if (!nvMediaIep_) {
    throw NvidiaGmslDriverRosFatalException("Unable to create NvMedia IEP.");
  }

  SetEncodeConfig();
  SetEncodeConfigRCParam();

  CHECK_NVMEDIA_ERROR_ROS(NvMediaIEPSetConfiguration(nvMediaIep_, &encodeConfig_))
}

NvMediaH264Encoder::~NvMediaH264Encoder()
{
  NvMediaIEPDestroy(nvMediaIep_);
}

void NvMediaH264Encoder::SetEncodeInitParams()
{
  encodeInitParams_.encodeWidth = 176;
  encodeInitParams_.encodeHeight = 144;
  encodeInitParams_.frameRateNum = 30;
  encodeInitParams_.frameRateDen = 1;
  encodeInitParams_.profile = 0;
  encodeInitParams_.level = 0;
  encodeInitParams_.maxNumRefFrames = 2;
  encodeInitParams_.enableExternalMEHints = NVMEDIA_FALSE;
  encodeInitParams_.useBFramesAsRef = 0;
  encodeInitParams_.enableAllIFrames = 0;
}

void NvMediaH264Encoder::SetEncodeConfig()
{
  encodeConfig_.pocType = NVMEDIA_ENCODE_H264_POC_TYPE_AUTOSELECT;
  encodeConfig_.bdirectMode = NVMEDIA_ENCODE_H264_BDIRECT_MODE_SPATIAL;
  encodeConfig_.adaptiveTransformMode = NVMEDIA_ENCODE_H264_ADAPTIVE_TRANSFORM_AUTOSELECT;
  encodeConfig_.disableDeblockingFilterIDC = 0;
  encodeConfig_.enableWeightedPrediction = 0;
  encodeConfig_.entropyCodingMode = NVMEDIA_ENCODE_H264_ENTROPY_CODING_MODE_CAVLC;
  encodeConfig_.features = 0b0001;
  encodeConfig_.gopLength = encodeInitParams_.frameRateNum / encodeInitParams_.frameRateDen;
  encodeConfig_.idrPeriod = encodeConfig_.gopLength;
  encodeConfig_.repeatSPSPPS = NVMEDIA_ENCODE_SPSPPS_REPEAT_IDR_FRAMES;
  encodeConfig_.quality = NVMEDIA_ENCODE_QUALITY_L1;
  encodeConfig_.numSliceCountMinus1 = 0;
  encodeConfig_.numMacroblocksPerSlice = 0;
  encodeConfig_.motionPredictionExclusionFlags = 0;
  encodeConfig_.intraRefreshCnt = 0;
  encodeConfig_.h264VUIParameters = encodeConfigH264VuiParams_;
}

void NvMediaH264Encoder::SetEncodeConfigRCParam()
{
  rcParams_.rateControlMode = NVMEDIA_ENCODE_PARAMS_RC_CBR;
  rcParams_.numBFrames = 0;
  rcParams_.params.cbr.averageBitRate = 2000000;
  rcParams_.params.cbr.vbvBufferSize = 0;
  rcParams_.params.cbr.vbvInitialDelay = 0;
  encodeConfig_.rcParams = rcParams_;
}

void NvMediaH264Encoder::SetEncodePicParams()
{
  encodePicParams_.pictureType = NVMEDIA_ENCODE_PIC_TYPE_AUTOSELECT;
  encodePicParams_.encodePicFlags = 0b0001;
  encodePicParams_.frameRateDen = encodeInitParams_.frameRateDen;
  encodePicParams_.frameRateNum = encodeInitParams_.frameRateNum;
  encodePicParams_.nextBFrames = 0;
  encodePicParams_.rcParams = encodeConfig_.rcParams;
}

void NvMediaH264Encoder::feed_frame(dwImageNvMedia* inNvMediaImage)
{
  if (!inNvMediaImage) {
    throw NvidiaGmslDriverRosFatalException("IEP H264 Feed frame : inNvMediaImage is False");
  }

  SetEncodePicParams();
  CHECK_NVMEDIA_ERROR_ROS(
      NvMediaIEPFeedFrame(nvMediaIep_, inNvMediaImage->img, nullptr, &encodePicParams_, NVMEDIA_ENCODER_INSTANCE_0));
}
