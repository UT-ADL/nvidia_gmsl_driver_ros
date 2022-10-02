// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#include "encoders/NvMediaJpgEncoder.h"

NvMediaJpgEncoder::NvMediaJpgEncoder(DriveworksApiWrapper* driveworksApiWrapper, int width, int height)
  : driveworksApiWrapper_(driveworksApiWrapper), width_(width), height_(height)
{
  nvmediaDevice_ = NvMediaDeviceCreate();
  nvMediaIjpe_ = NvMediaIJPECreate(nvmediaDevice_, surfaceType_, (uint8_t)1, BUFFER_SIZE);

  if (!nvmediaDevice_ || !nvMediaIjpe_) {
    throw NvidiaGmslDriverRosFatalException("Unable to create NvMedia device or Encoder!");
  }

  CHK_DW(dwImage_create(&imgYuv420Pi_, camera_common::get_yuv420_pitch_img_prop(width_, height_),
                        driveworksApiWrapper_->context_handle_));
}

NvMediaJpgEncoder::~NvMediaJpgEncoder()
{
  NvMediaIJPEDestroy(nvMediaIjpe_);
  NvMediaDeviceDestroy(nvmediaDevice_);
}

void NvMediaJpgEncoder::feed_frame(const dwImageHandle_t* input)
{
  dwImage_copyConvert(imgYuv420Pi_, *input, driveworksApiWrapper_->context_handle_);
  CHK_DW(dwImage_getNvMedia(&image_nvmedia_, imgYuv420Pi_));
  CHK_NVM(NvMediaIJPEFeedFrame(nvMediaIjpe_, image_nvmedia_->img, 70));
}

bool NvMediaJpgEncoder::bits_available()
{
  nvMediaStatus_ =
      NvMediaIJPEBitsAvailable(nvMediaIjpe_, &numBytesAvailable_, NVMEDIA_ENCODE_BLOCKING_TYPE_IF_PENDING, 0);

  if (nvMediaStatus_ == NVMEDIA_STATUS_OK) {
    return true;
  }

  if (nvMediaStatus_ == NVMEDIA_STATUS_BAD_PARAMETER) {
    throw NvidiaGmslDriverRosFatalException("Bad parameters for NvMediaIJPEBitsAvailable");
  }

  return bits_available();
}

void NvMediaJpgEncoder::pull_bits()
{
  CHK_NVM(NvMediaIJPEGetBits(nvMediaIjpe_, &numBytesAvailable_, buffer_.data(), 0));
}

uint8_t* NvMediaJpgEncoder::get_buffer()
{
  return buffer_.data();
}

uint32_t NvMediaJpgEncoder::get_num_bytes_available() const
{
  return numBytesAvailable_;
}
