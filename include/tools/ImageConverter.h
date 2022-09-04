// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#pragma once

#include "DriveworksApiWrapper.h"
#include "framework/Checks.hpp"

#include <dw/image/Image.h>

class ImageConverter
{
public:
  ImageConverter(DriveworksApiWrapper* driveworksApiWrapper, uint32_t width, uint32_t height,
                 dwImageType outputImageType, dwImageFormat outputImageFormat,
                 dwImageMemoryType outputImageMemoryLayout);

  /**
   * Converts an image according to the initialization parameters.
   * @param input Ptr to the input dwImageHandle_t.
   * @return std::unique_ptr<dwImageHandle_t>.
   */
  [[nodiscard]] std::unique_ptr<dwImageHandle_t> convert(dwImageHandle_t* input);

private:
  DriveworksApiWrapper* driveworksApiWrapper_;
  dwImageProperties imageProperties_{};
  std::unique_ptr<dwImageHandle_t> outputImage_;
};
