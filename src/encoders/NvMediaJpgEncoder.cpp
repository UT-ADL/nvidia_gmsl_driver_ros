// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#include "encoders/NvMediaJpgEncoder.h"

NvMediaJpgEncoder::NvMediaJpgEncoder(const NvMediaSurfaceType* surfaceType) : surfaceType_(*surfaceType)
{
  jpegImage_ = std::make_unique<uint8_t[]>(MAX_JPG_BYTES);

  nvmediaDevice_ = NvMediaDeviceCreate();
  nvMediaIjpe_ = NvMediaIJPECreate(nvmediaDevice_, surfaceType_, (uint8_t)1, MAX_JPG_BYTES);

  if (!nvmediaDevice_ || !nvMediaIjpe_) {
    throw NvidiaGmslDriverRosFatalException("Unable to create NvMedia device or Encoder!");
  }
}

NvMediaJpgEncoder::~NvMediaJpgEncoder()
{
  NvMediaIJPEDestroy(nvMediaIjpe_);
  NvMediaDeviceDestroy(nvmediaDevice_);
}

void NvMediaJpgEncoder::feed_frame(dwImageNvMedia* inNvMediaImage) const
{
  if (!inNvMediaImage) {
    throw NvidiaGmslDriverRosFatalException("IJPE Feed frame : inNvMediaImage is False");
  }

  CHECK_NVMEDIA_ERROR_ROS_FATAL(NvMediaIJPEFeedFrame(nvMediaIjpe_, inNvMediaImage->img, 70))
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

void NvMediaJpgEncoder::pull_bits(){ CHECK_NVMEDIA_ERROR_ROS_FATAL(NvMediaIJPEGetBits(nvMediaIjpe_, &countByteJpeg_,
                                                                                      jpegImage_.get(), 0)) }

uint8_t* NvMediaJpgEncoder::get_image()
{
  return jpegImage_.get();
}

uint32_t NvMediaJpgEncoder::get_count_bytes() const
{
  return countByteJpeg_;
}
