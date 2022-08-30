// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#include "tools/ImageConverter.h"
ImageConverter::ImageConverter(std::shared_ptr<DriveworksApiWrapper> driveworksApiWrapper, uint32_t width,
                               uint32_t height, dwImageType outputImageType, dwImageFormat outputImageFormat,
                               dwImageMemoryType outputImageMemoryLayout)
  : driveworksApiWrapper_(driveworksApiWrapper)
{
  imageProperties_.type = outputImageType;
  imageProperties_.format = outputImageFormat;
  imageProperties_.width = width;
  imageProperties_.height = height;
  imageProperties_.memoryLayout = outputImageMemoryLayout;
}

std::unique_ptr<dwImageHandle_t> ImageConverter::convert(dwImageHandle_t* input)
{
  outputImage_ = std::make_unique<dwImageHandle_t>();
  CHECK_DW_ERROR_ROS(dwImage_create(outputImage_.get(), imageProperties_, driveworksApiWrapper_->context_handle_));
  CHECK_DW_ERROR_ROS(dwImage_copyConvert(*outputImage_, *input, driveworksApiWrapper_->context_handle_));
  return std::move(outputImage_);
}