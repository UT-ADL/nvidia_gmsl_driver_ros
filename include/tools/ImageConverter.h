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
   * todo
   */
  dwImageHandle_t* convert(dwImageHandle_t* input, dwImageHandle_t* output);

private:
  DriveworksApiWrapper* driveworksApiWrapper_;
  dwImageProperties imageProperties_{};
  std::unique_ptr<dwImageHandle_t> outputImage_;
};
