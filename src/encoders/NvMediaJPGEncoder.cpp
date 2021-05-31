// Created by Maxandre Ogeret.
// (c) 2021 University of Tartu - Autonomous Driving Lab.

#include "encoders/NvMediaJPGEncoder.h"

NvMediaJPGEncoder::NvMediaJPGEncoder(NvMediaSurfaceType surfaceType): surfaceType_(surfaceType)
{
  // Nvmedia init
  nvmediaDevice_ = NvMediaDeviceCreate();
  nvMediaIjpe_ = NvMediaIJPECreate(nvmediaDevice_, surfaceType_, (uint8_t)1, maxJpegBytes_);

}

NvMediaJPGEncoder::~NvMediaJPGEncoder()
{
  NvMediaIJPEDestroy(nvMediaIjpe_);
  NvMediaDeviceDestroy(nvmediaDevice_);
}

void NvMediaJPGEncoder::feed_frame(NvMediaImage* inNvMediaImage)
{
  CHECK_NVMEDIA_ERROR_ROS_FATAL(NvMediaIJPEFeedFrame(nvMediaIjpe_, inNvMediaImage, 70))
}

bool NvMediaJPGEncoder::bits_available()
{
  nvMediaStatus_ = NvMediaIJPEBitsAvailable(nvMediaIjpe_, &countByteJpeg_, NVMEDIA_ENCODE_BLOCKING_TYPE_NEVER, 0);

  if (nvMediaStatus_ == NVMEDIA_STATUS_OK) {
    return true;
  }

  if (nvMediaStatus_ == NVMEDIA_STATUS_BAD_PARAMETER) {
    throw SekonixDriverFatalException("Bad parameters for NvMediaIJPEBitsAvailable");
  }

  return false;
}

uint8_t* NvMediaJPGEncoder::get_bits()
{
  return jpegImage_.get();
}

uint32_t NvMediaJPGEncoder::get_count_bytes()
{
  return countByteJpeg_;
}