// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#include "encoders/NvMediaJpgEncoder.h"

NvMediaJpgEncoder::NvMediaJpgEncoder(DriveworksApiWrapper* driveworksApiWrapper, int width, int height)
  : driveworksApiWrapper_(driveworksApiWrapper), width_(width), height_(height)
{
  jpegImage_ = std::make_unique<uint8_t[]>(MAX_JPG_BYTES);

  nvmediaDevice_ = NvMediaDeviceCreate();
  nvMediaIjpe_ = NvMediaIJPECreate(nvmediaDevice_, surfaceType_, (uint8_t)1, MAX_JPG_BYTES);

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

bool NvMediaJpgEncoder::wait_for_bits()
{
  nvMediaStatus_ = NvMediaIJPEBitsAvailable(nvMediaIjpe_, &countByteJpeg_, NVMEDIA_ENCODE_BLOCKING_TYPE_IF_PENDING, 0);

  if (nvMediaStatus_ == NVMEDIA_STATUS_OK) {
    return true;
  }

  if (nvMediaStatus_ == NVMEDIA_STATUS_BAD_PARAMETER) {
    throw NvidiaGmslDriverRosFatalException("Bad parameters for NvMediaIJPEBitsAvailable");
  }

  return wait_for_bits();
}

void NvMediaJpgEncoder::pull_bits()
{
  CHK_NVM(NvMediaIJPEGetBits(nvMediaIjpe_, &countByteJpeg_, jpegImage_.get(), 0));
}

uint8_t* NvMediaJpgEncoder::get_image()
{
  return jpegImage_.get();
}

uint32_t NvMediaJpgEncoder::get_count_bytes() const
{
  return countByteJpeg_;
}
