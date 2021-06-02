// Created by Maxandre Ogeret.
// (c) 2021 University of Tartu - Autonomous Driving Lab.

#include "encoders/NvMediaJPGEncoder.h"

NvMediaJPGEncoder::NvMediaJPGEncoder(NvMediaSurfaceType* surfaceType): surfaceType_(*surfaceType)
{
  jpegImage_ = std::make_unique<uint8_t[]>(maxJpegBytes_);

  // Nvmedia init
  nvmediaDevice_ = NvMediaDeviceCreate();
  nvMediaIjpe_ = NvMediaIJPECreate(nvmediaDevice_, surfaceType_, (uint8_t)1, maxJpegBytes_);

  if (!nvmediaDevice_ || !nvmediaDevice_) {
    throw SekonixDriverFatalException("Unable to create NvMedia device or Encoder!");
  }
}

NvMediaJPGEncoder::~NvMediaJPGEncoder()
{
  NvMediaIJPEDestroy(nvMediaIjpe_);
  NvMediaDeviceDestroy(nvmediaDevice_);
}

void NvMediaJPGEncoder::feed_frame(dwImageNvMedia* inNvMediaImage)
{
  if (!inNvMediaImage) {
    throw SekonixDriverFatalException("IJPE Feed frame : inNvMediaImage is False");
  }

  CHECK_NVMEDIA_ERROR_ROS_FATAL(NvMediaIJPEFeedFrame(nvMediaIjpe_, inNvMediaImage->img, 70))
}

bool NvMediaJPGEncoder::bits_available()
{
  nvMediaStatus_ = NvMediaIJPEBitsAvailable(nvMediaIjpe_, &countByteJpeg_, NVMEDIA_ENCODE_BLOCKING_TYPE_IF_PENDING, 0);

  if (nvMediaStatus_ == NVMEDIA_STATUS_OK) {
    return true;
  }

  if (nvMediaStatus_ == NVMEDIA_STATUS_BAD_PARAMETER) {
    throw SekonixDriverFatalException("Bad parameters for NvMediaIJPEBitsAvailable");
  }

  return false;
}

void NvMediaJPGEncoder::pull_bits() {
  CHECK_NVMEDIA_ERROR_ROS_FATAL(NvMediaIJPEGetBits(nvMediaIjpe_, &countByteJpeg_, jpegImage_.get(), 0));
}

std::shared_ptr<uint8_t[]> NvMediaJPGEncoder::get_image()
{
  return jpegImage_;
}

uint32_t NvMediaJPGEncoder::get_count_bytes()
{
  return countByteJpeg_;
}
