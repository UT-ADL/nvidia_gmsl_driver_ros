// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#include "encoders/NvMediaH264Encoder.h"

NvMediaH264Encoder::NvMediaH264Encoder(const NvMediaSurfaceType* surfaceType) : surfaceType_(*surfaceType)
{
  NvMediaIEPGetVersion(&nvMediaVersion_);
  nvMediaDevice_ = NvMediaDeviceCreate();
  SetEncodeInitParams();

  nvMediaIep_ = NvMediaIEPCreate(nvMediaDevice_, NVMEDIA_IMAGE_ENCODE_H264, initializeParams_, surfaceType_, NULL, 16,
                                 NVMEDIA_ENCODER_INSTANCE_AUTO);

  if (!nvMediaIep_) {
    throw NvidiaGmslDriverRosFatalException("Unable to create NvMedia IEP.");
  }

  SetEncodeConfig();
  SetEncodeConfigRCParam();

  CHECK_NVMEDIA_ERROR_ROS_FATAL(NvMediaIEPSetConfiguration(nvMediaIep_, encodeConfig_))
}

NvMediaH264Encoder::~NvMediaH264Encoder()
{
  NvMediaIEPDestroy(nvMediaIep_);
}

void NvMediaH264Encoder::SetEncodeInitParams()
{
  initializeParams_->encodeWidth = 176;
  initializeParams_->encodeHeight = 144;
  initializeParams_->frameRateNum = 30;
  initializeParams_->frameRateDen = 1;
  initializeParams_->profile = 0;
  initializeParams_->level = 0;
  initializeParams_->maxNumRefFrames = 2;
  initializeParams_->enableExternalMEHints = NVMEDIA_FALSE;
  initializeParams_->useBFramesAsRef = 0;
  initializeParams_->enableAllIFrames = 0;
}

void NvMediaH264Encoder::SetEncodeConfig()
{
  encodeConfig_->pocType = NVMEDIA_ENCODE_H264_POC_TYPE_AUTOSELECT;
  encodeConfig_->bdirectMode = NVMEDIA_ENCODE_H264_BDIRECT_MODE_SPATIAL;
  encodeConfig_->adaptiveTransformMode = NVMEDIA_ENCODE_H264_ADAPTIVE_TRANSFORM_AUTOSELECT;
  encodeConfig_->disableDeblockingFilterIDC = 0;
  encodeConfig_->enableWeightedPrediction = 0;
  encodeConfig_->entropyCodingMode = NVMEDIA_ENCODE_H264_ENTROPY_CODING_MODE_CAVLC;
  encodeConfig_->features = 0;
  encodeConfig_->gopLength = initializeParams_->frameRateNum / initializeParams_->frameRateDen;
  encodeConfig_->idrPeriod = encodeConfig_->gopLength;
  encodeConfig_->repeatSPSPPS = NVMEDIA_ENCODE_SPSPPS_REPEAT_IDR_FRAMES;
  encodeConfig_->quality = NVMEDIA_ENCODE_QUALITY_L1;
  encodeConfig_->numSliceCountMinus1 = 0;
  encodeConfig_->numMacroblocksPerSlice = 0;
  encodeConfig_->motionPredictionExclusionFlags = 0;
  encodeConfig_->intraRefreshCnt = 0;
  encodeConfig_->h264VUIParameters = encodeConfigH264VuiParams_;
}

void NvMediaH264Encoder::SetEncodeConfigRCParam()
{
  rcParams_->rateControlMode = NVMEDIA_ENCODE_PARAMS_RC_CBR;
  rcParams_->numBFrames = 0;
  rcParams_->params.cbr.averageBitRate = 2000000;
  rcParams_->params.cbr.vbvBufferSize = 0;
  rcParams_->params.cbr.vbvInitialDelay = 0;
  encodeConfig_->rcParams = *rcParams_;
}

void NvMediaH264Encoder::SetEncodePicParams()
{
  encodePicParams_->pictureType = NVMEDIA_ENCODE_PIC_TYPE_AUTOSELECT;
  // todo: Finish the encode pic param.
  // See samples nvmimg_encode.c & image_encoder.c
}

void NvMediaH264Encoder::feed_frame(dwImageNvMedia* inNvMediaImage)
{
  if (!inNvMediaImage) {
    throw NvidiaGmslDriverRosFatalException("IEP H264 Feed frame : inNvMediaImage is False");
  }

  SetEncodePicParams();
  NvMediaIEPFeedFrame(nvMediaIep_, inNvMediaImage->img, nullptr, encodePicParams_, NVMEDIA_ENCODER_INSTANCE_AUTO);
}
