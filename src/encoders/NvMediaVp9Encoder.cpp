// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#include "encoders/NvMediaVp9Encoder.h"

NvMediaVp9Encoder::NvMediaVp9Encoder(DriveworksApiWrapper* driveworksApiWrapper, int width, int height, int framerate,
                                     int bitrate)
  : driveworksApiWrapper_(driveworksApiWrapper)
  , width_(width)
  , height_(height)
  , framerate_(framerate)
  , bitrate_(bitrate)
{
  NvMediaIEPGetVersion(&nvMediaVersion_);
  nvMediaDevice_ = NvMediaDeviceCreate();

  if (!nvMediaDevice_) {
    throw NvidiaGmslDriverRosFatalException("Unable to create NvMedia device.");
  }

  set_encode_init_params();

  nvMediaIep_ = NvMediaIEPCreate(nvMediaDevice_, NVMEDIA_IMAGE_ENCODE_VP9, &encodeInitParams_, surfaceType_, '\0', 100,
                                 NVMEDIA_ENCODER_INSTANCE_0);

  if (!nvMediaIep_) {
    throw NvidiaGmslDriverRosFatalException("Unable to create NvMedia IEP.");
  }

  set_encode_config();
  set_encode_config_rcparam();
  set_encode_pic_params();

  CHK_NVM(NvMediaIEPSetConfiguration(nvMediaIep_, &encodeConfig_));

  nvMediaBitstreamBuffer_.bitstream = buffer_.data();

  dwImage_create(&imgYuv420Bl_, camera_common::get_yuv420_block_img_prop(width_, height_),
                 driveworksApiWrapper_->context_handle_);
}

NvMediaVp9Encoder::~NvMediaVp9Encoder()
{
  NvMediaIEPDestroy(nvMediaIep_);
  dwImage_destroy(imgYuv420Bl_);
}

void NvMediaVp9Encoder::set_encode_init_params()
{
  encodeInitParams_.encodeWidth = static_cast<uint16_t>(width_);
  encodeInitParams_.encodeHeight = static_cast<uint16_t>(height_);
  encodeInitParams_.frameRateNum = framerate_;
  encodeInitParams_.frameRateDen = 1;
  encodeInitParams_.maxNumRefFrames = 2;
  encodeInitParams_.enableExternalMEHints = NVMEDIA_FALSE;
}

void NvMediaVp9Encoder::set_encode_config()
{
  encodeConfig_.features = 0;
  encodeConfig_.gopLength = encodeInitParams_.frameRateNum / encodeInitParams_.frameRateDen;
  encodeConfig_.idrPeriod = encodeConfig_.gopLength;
}

void NvMediaVp9Encoder::set_encode_config_rcparam()
{
  rcParams_.rateControlMode = NVMEDIA_ENCODE_PARAMS_RC_CBR;
  rcParams_.numBFrames = 0;
  rcParams_.params.cbr.averageBitRate = bitrate_;
  rcParams_.params.cbr.vbvBufferSize = 0;
  rcParams_.params.cbr.vbvInitialDelay = 0;
  encodeConfig_.rcParams = rcParams_;
}

void NvMediaVp9Encoder::set_encode_pic_params()
{
  encodePicParams_.pictureType = NVMEDIA_ENCODE_PIC_TYPE_AUTOSELECT;
  encodePicParams_.nextBFrames = 0;
  encodePicParams_.encodePicFlags = 0b0001;
  encodePicParams_.rcParams = encodeConfig_.rcParams;
}

void NvMediaVp9Encoder::feed_frame(const dwImageHandle_t* input)
{
  dwImage_copyConvert(imgYuv420Bl_, *input, driveworksApiWrapper_->context_handle_);
  CHK_DW(dwImage_getNvMedia(&image_nvmedia_, imgYuv420Bl_));

  nvMediaStatus_ =
      NvMediaIEPFeedFrame(nvMediaIep_, image_nvmedia_->img, nullptr, &encodePicParams_, NVMEDIA_ENCODER_INSTANCE_0);

  if (nvMediaStatus_ == NVMEDIA_STATUS_INSUFFICIENT_BUFFERING) {
    return;
  }
  CHK_NVM(nvMediaStatus_);
}

bool NvMediaVp9Encoder::bits_available()
{
  nvMediaStatus_ = NvMediaIEPBitsAvailable(nvMediaIep_, &numBytesAvailable_, NVMEDIA_ENCODE_BLOCKING_TYPE_IF_PENDING,
                                           MILLISECOND_TIMEOUT_);

  if (nvMediaStatus_ == NVMEDIA_STATUS_OK && numBytesAvailable_ > 0) {
    return true;
  }

  if (nvMediaStatus_ == NVMEDIA_STATUS_BAD_PARAMETER) {
    throw NvidiaGmslDriverRosFatalException("Bad parameters for NvMediaIEPBitsAvailable");
  }
  return false;
}

void NvMediaVp9Encoder::pull_bits()
{
  nvMediaStatus_ = NvMediaIEPGetBitsEx(nvMediaIep_, &numBytesAvailable_, 1, &nvMediaBitstreamBuffer_, nullptr);

  switch (nvMediaStatus_) {
    case NVMEDIA_STATUS_OK:
    case NVMEDIA_STATUS_INSUFFICIENT_BUFFERING:
    case NVMEDIA_STATUS_PENDING:
      return;
    default:
      std::string msg = "Error while pulling data from vp9 encoder. Error ID: " + std::to_string(nvMediaStatus_) +
                        ", " + NVMEDIA_ERROR_TO_STRING.at(nvMediaStatus_) + ".";
      ROS_FATAL_STREAM(msg);
      throw NvidiaGmslDriverRosFatalException(msg);
  }
}

uint8_t* NvMediaVp9Encoder::get_buffer()
{
  return buffer_.data();
}

uint32_t NvMediaVp9Encoder::get_num_bytes_available() const
{
  return numBytesAvailable_;
}
