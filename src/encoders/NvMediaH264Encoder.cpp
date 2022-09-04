// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#include "encoders/NvMediaH264Encoder.h"

NvMediaH264Encoder::NvMediaH264Encoder(const NvMediaSurfaceType* surfaceType, int width, int height, int framerate,
                                       int bitrate)
  : surfaceType_(*surfaceType), width_(width), height_(height), framerate_(framerate), bitrate_(bitrate)
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

  CHK_NVM(NvMediaIEPSetConfiguration(nvMediaIep_, &encodeConfig_));

  nvMediaBitstreamBuffer_.bitstream = buffer_.data();
}

NvMediaH264Encoder::~NvMediaH264Encoder()
{
  NvMediaIEPDestroy(nvMediaIep_);
}

void NvMediaH264Encoder::SetEncodeInitParams()
{
  encodeInitParams_.encodeWidth = static_cast<uint16_t>(width_);
  encodeInitParams_.encodeHeight = static_cast<uint16_t>(height_);
  encodeInitParams_.frameRateNum = framerate_;
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
  rcParams_.params.cbr.averageBitRate = bitrate_;
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

void NvMediaH264Encoder::feed_frame(const dwImageNvMedia* inNvMediaImage)
{
  if (!inNvMediaImage) {
    throw NvidiaGmslDriverRosFatalException("IEP H264 Feed frame : inNvMediaImage is False");
  }

  SetEncodePicParams();
  CHK_NVM(
      NvMediaIEPFeedFrame(nvMediaIep_, inNvMediaImage->img, nullptr, &encodePicParams_, NVMEDIA_ENCODER_INSTANCE_0));
}

bool NvMediaH264Encoder::bits_available()
{
  nvMediaStatus_ =
      NvMediaIEPBitsAvailable(nvMediaIep_, &numBytesAvailable_, NVMEDIA_ENCODE_BLOCKING_TYPE_IF_PENDING, 0);

  if (nvMediaStatus_ == NVMEDIA_STATUS_OK && numBytesAvailable_ > 0) {
    std::cout << "numBytesAvailable_ " << (int)numBytesAvailable_ << " \n";
    return true;
  }

  if (nvMediaStatus_ == NVMEDIA_STATUS_BAD_PARAMETER) {
    throw NvidiaGmslDriverRosFatalException("Bad parameters for NvMediaIEPBitsAvailable");
  }

  return false;
}

void NvMediaH264Encoder::pull_bits()
{
  nvMediaStatus_ = NvMediaIEPGetBitsEx(nvMediaIep_, &numBytesAvailable_, 1, &nvMediaBitstreamBuffer_, nullptr);

  if (nvMediaStatus_ == NVMEDIA_STATUS_OK) {
    std::cout << "# pull_bits NVMEDIA_STATUS_OK\n";
    std::cout << "# pull_bits bitstreamBytes: " << (int)(nvMediaBitstreamBuffer_.bitstreamBytes) << "\n";
    std::cout << "# pull_bits numBytesAvailable_: " << (int)numBytesAvailable_ << "\n";

    return;
  }

  if (nvMediaStatus_ == NVMEDIA_STATUS_INSUFFICIENT_BUFFERING) {
    std::cout << "# pull_bits NVMEDIA_STATUS_INSUFFICIENT_BUFFERING\n";
    return;
  }

  if (nvMediaStatus_ == NVMEDIA_STATUS_PENDING) {
    std::cout << "# pull_bits NVMEDIA_STATUS_PENDING\n";
    return;
  }

  std::cout << "# pull_bits OTHER " << (int)nvMediaStatus_ << "\n";
}

std::array<uint8_t, BUFFER_SIZE>* NvMediaH264Encoder::get_buffer()
{
  return &buffer_;
}

uint32_t NvMediaH264Encoder::getNumBytes_() const
{
  return numBytesAvailable_;
}
