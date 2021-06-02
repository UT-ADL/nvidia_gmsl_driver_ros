// Created by Maxandre Ogeret.
// (c) 2021 University of Tartu - Autonomous Driving Lab.

#ifndef SEKONIX_CAMERA_UT_NVMEDIAJPGENCODER_H
#define SEKONIX_CAMERA_UT_NVMEDIAJPGENCODER_H

#include <nvmedia_ijpe.h>
#include <memory>

#include "framework/Checks.hpp"
#include "exceptions/SekonixDriverFatalException.h"
#include <dw/image/Image.h>

class NvMediaJPGEncoder
{
public:
  /**
   * @brief Constructor
  */
  NvMediaJPGEncoder(NvMediaSurfaceType* surfaceType);

  /**
   * @brief Destructor
   */
  virtual ~NvMediaJPGEncoder();

  /**
   * @brief Feeds a frame into the encoder
   */
  void feed_frame(dwImageNvMedia* inNvMediaImage);

  /**
   * @brief Returns true if encoded bits are available in the encoder memory
   */
  bool bits_available();

  /**
   * @brief Pull the bits from the encoder memory to the local image. Must be called if bits are available
   */
  void pull_bits();

  /**
   * @brief Returns a shared ptr to the image bits pulled from the encoder
   */
  std::shared_ptr<uint8_t[]> get_image();

  /**
   * @brief Returns the size in bytes of the current image.
   */
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