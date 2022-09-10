// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#include "tools/ImageConverter.h"

ImageConverter::ImageConverter(DriveworksApiWrapper* driveworksApiWrapper, uint32_t width, uint32_t height,
                               dwImageType outputImageType, dwImageFormat outputImageFormat,
                               dwImageMemoryType outputImageMemoryLayout)
  : driveworksApiWrapper_(driveworksApiWrapper)
{
  imageProperties_.type = outputImageType;
  imageProperties_.format = outputImageFormat;
  imageProperties_.width = width;
  imageProperties_.height = height;
  imageProperties_.memoryLayout = outputImageMemoryLayout;
}

dwImageHandle_t* ImageConverter::convert(dwImageHandle_t* input, dwImageHandle_t* output)
{
  CHK_DW(dwImage_copyConvert(*output, *input, driveworksApiWrapper_->context_handle_));
}

// const dwImageProperties& ImageConverter::getOuputImageProperties() const
//{
//   return imageProperties_;
// }
