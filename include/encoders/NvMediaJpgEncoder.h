// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#pragma once

#include <dw/image/Image.h>
#include <nvmedia_ijpe.h>

#include <memory>

#include "DriveworksApiWrapper.h"
#include "cameras/CameraCommon.h"
#include "framework/Checks.hpp"

/**
 * @brief Wrapper around NVIDIA Media Interface: Image JPEG Encode Processing API.
 * @see
 * https://docs.nvidia.com/drive/drive-os-5.2.0.0L/drive-os/DRIVE_OS_Linux_SDK_Development_Guide/baggage/nvmedia__ijpe_8h.html
 */
class NvMediaJpgEncoder
{
public:
  /**
   * @brief Constructor.
   * @throws NvidiaGmslDriverRosFatalException
   */
  NvMediaJpgEncoder(DriveworksApiWrapper* driveworksApiWrapper, int width, int height);

  /**
   * @brief Destructor.
   */
  virtual ~NvMediaJpgEncoder();

  /**
   * @brief Feeds a frame into the encoder.
   */
  void feed_frame(const dwImageHandle_t* input);

  /**
   * @brief Polls the encoder and waits for encoded bits availability. Has no timeout.
   * @attention Prerequisite : feed_frame()
   * @throws NvidiaGmslDriverRosFatalException
   */
  bool bits_available();

  /**
   * @brief Returns a ptr to the image bits pulled from the encoder.
   */
  void pull_bits();

  /**
   * @brief Returns a ptr to the image bits pulled from the encoder.
   */
  [[nodiscard]] uint8_t* get_buffer();

  /**
   * @brief Returns the size in bytes of the data stored in the buffer.
   */
  [[nodiscard]] uint32_t get_num_bytes_available() const;

private:
  /** 4 MiB buffer. */
  static constexpr int BUFFER_SIZE = 4194304;

  DriveworksApiWrapper* driveworksApiWrapper_;

  NvMediaDevice* nvmediaDevice_ = nullptr;
  NvMediaIJPE* nvMediaIjpe_ = nullptr;
  NvMediaSurfaceType surfaceType_ = camera_common::get_yuv420_pitch_img_surface();
  NvMediaStatus nvMediaStatus_;

  dwImageHandle_t imgYuv420Pi_ = DW_NULL_HANDLE;
  dwImageNvMedia* image_nvmedia_ = nullptr;

  uint32_t numBytesAvailable_;
  std::array<uint8_t, BUFFER_SIZE> buffer_;

  int width_;
  int height_;
};
