// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#pragma once

#include <dw/image/Image.h>
#include <nvmedia_iep.h>

#include "DriveworksApiWrapper.h"
#include "cameras/CameraCommon.h"
#include "exceptions/NvidiaGmslDriverRosFatalException.h"
#include "framework/Checks.hpp"

/**
 * @brief Wrapper around NVIDIA Media Interface: NvMedia Image Encode Processing API for H265.
 * @see
 * https://docs.nvidia.com/drive/drive-os-5.2.0.0L/drive-os/DRIVE_OS_Linux_SDK_Development_Guide/baggage/nvmedia__iep_8h.html
 */
class NvMediaH265Encoder
{
public:
  /**
   * Constructor.
   * @throws NvidiaGmslDriverRosFatalException
   */
  NvMediaH265Encoder(DriveworksApiWrapper* driveworksApiWrapper, int width, int height, int framerate, int bitrate);

  /**
   * Destructor.
   */
  virtual ~NvMediaH265Encoder();

  /**
   * @brief Feeds a frame into the encoder.
   */
  void feed_frame(const dwImageHandle_t* input);

  /**
   * @brief Polls the encoder and waits for encoded bits availability. Has a 1 frame timeout.
   * @attention Prerequisite : feed_frame()
   * @throws NvidiaGmslDriverRosFatalException
   */
  bool bits_available();

  /**
   * @brief Pull the bits from the encoder memory to the local image.
   * @attention Prerequisite : bits_available()
   * @throws NvidiaGmslDriverRosFatalException
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
  /** 1 MiB buffer. */
  static constexpr int BUFFER_SIZE = 1048576;  //
  /** bits_available timeout (1 frame worth of time). */
  const uint32_t MILLISECOND_TIMEOUT_ = static_cast<uint32_t>(std::ceil(1000.0 / framerate_));

  DriveworksApiWrapper* driveworksApiWrapper_;

  NvMediaVersion nvMediaVersion_;
  NvMediaDevice* nvMediaDevice_;
  NvMediaEncodeInitializeParamsH265 encodeInitParams_{};
  NvMediaEncodeConfigH265 encodeConfig_{};
  NvMediaEncodeConfigH265VUIParams* encodeConfigH265VuiParams_{};
  NvMediaEncodeRCParams rcParams_{};
  NvMediaEncodePicParamsH265 encodePicParams_{};
  NvMediaSurfaceType surfaceType_ = camera_common::get_yuv420_block_img_surface();
  NvMediaIEP* nvMediaIep_;
  NvMediaStatus nvMediaStatus_;

  dwImageHandle_t imgYuv420Bl_ = DW_NULL_HANDLE;
  dwImageNvMedia* image_nvmedia_ = nullptr;

  // Buffer
  std::array<uint8_t, BUFFER_SIZE> buffer_;
  NvMediaBitstreamBuffer nvMediaBitstreamBuffer_{ buffer_.data(), 0, BUFFER_SIZE };

  uint32_t numBytesAvailable_;

  // config
  int width_;
  int height_;
  int framerate_;
  int bitrate_;

  /**
   * @brief Sets the encoder initialization parameters.
   */
  void set_encode_init_params();

  /**
   * @brief Sets the encoder configuration.
   */
  void set_encode_config();

  /**
   * @brief Sets the encoder rate control parameters to use constant bitrate.
   */
  void set_encode_config_rcparam();

  /**
   * @brief Sets the encoder picture parameters.
   */
  void set_encode_pic_params();
};
