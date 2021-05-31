// Created by Maxandre Ogeret.
// (c) 2021 University of Tartu - Autonomous Driving Lab.

#ifndef SEKONIX_CAMERA_UT_NVMEDIAJPGENCODER_H
#define SEKONIX_CAMERA_UT_NVMEDIAJPGENCODER_H

#include <nvmedia_ijpe.h>
#include <memory>

#include "framework/Checks.hpp"
#include "exceptions/SekonixDriverFatalException.h"

class NvMediaJPGEncoder
{
public:
  virtual ~NvMediaJPGEncoder();
  NvMediaJPGEncoder(NvMediaSurfaceType surfaceType);
  void feed_frame(NvMediaImage* inNvMediaImage);
  bool bits_available();
  uint8_t* get_bits();
  uint32_t get_count_bytes();

private:
  NvMediaDevice* nvmediaDevice_ = nullptr;
  NvMediaIJPE* nvMediaIjpe_ = nullptr;
  NvMediaSurfaceType surfaceType_;
  NvMediaStatus nvMediaStatus_;
  uint32_t countByteJpeg_;

  std::shared_ptr<uint8_t[]> jpegImage_;
  const uint32_t maxJpegBytes_ = 3 * 1290 * 1208;
};

#endif  // SEKONIX_CAMERA_UT_NVMEDIAJPGENCODER_H
