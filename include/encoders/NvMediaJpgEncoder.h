// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#pragma once

#include <memory>

#include <nvmedia_ijpe.h>
#include <dw/image/Image.h>

#include "framework/Checks.hpp"
#include "cameras/CameraCommon.h"
#include "DriveworksApiWrapper.h"

static constexpr uint32_t MAX_JPG_BYTES = 3 * 1290 * 1208;

class NvMediaJpgEncoder
{
public:
  /**
   * @brief Constructor
   */
  NvMediaJpgEncoder(DriveworksApiWrapper* driveworksApiWrapper, int width, int height);

  /**
   * @brief Destructor
   */
  virtual ~NvMediaJpgEncoder();

  /**
   * @brief Feeds a frame into the encoder
   */
  void feed_frame(const dwImageHandle_t* input);

  /**
   * @brief Returns true if encoded bits are available in the encoder memory, otherwise waits. Has to be called after
   * feed_frame
   */
  bool wait_for_bits();

  /**
   * @brief Pull the bits from the encoder memory to the local image. Must be called after bits_available
   */
  void pull_bits();

  /**
   * @brief Returns a ptr to the image bits pulled from the encoder
   */
  uint8_t* get_image();

  /**
   * @brief Returns the size in bytes of the current image.
   */
  [[nodiscard]] uint32_t get_count_bytes() const;

private:
  DriveworksApiWrapper* driveworksApiWrapper_;

  NvMediaDevice* nvmediaDevice_ = nullptr;
  NvMediaIJPE* nvMediaIjpe_ = nullptr;
  NvMediaSurfaceType surfaceType_ = camera_common::get_yuv420_pitch_img_surface();
  NvMediaStatus nvMediaStatus_;

  dwImageHandle_t imgYuv420Pi_ = DW_NULL_HANDLE;
  dwImageNvMedia* image_nvmedia_ = nullptr;

  uint32_t countByteJpeg_;
  std::unique_ptr<uint8_t[]> jpegImage_;
  int width_;
  int height_;
};
