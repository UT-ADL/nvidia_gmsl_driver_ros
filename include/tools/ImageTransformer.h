// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#pragma once

#include <dw/imageprocessing/geometry/imagetransformation/ImageTransformation.h>
#include <dw/sensors/camera/Camera.h>
#include <dw/image/Image.h>

#include "DriveworksApiWrapper.h"
#include "framework/Checks.hpp"
#include "tools/ImageConverter.h"
#include "cameras/CameraCommon.h"

class ImageTransformer
{
public:
  ImageTransformer(DriveworksApiWrapper* driveworksApiWrapper);

  /**
   * todo
   */
  void transform_image(dwImageHandle_t* input, dwImageHandle_t* output);

  virtual ~ImageTransformer();

private:
  DriveworksApiWrapper* driveworksApiWrapper_;
  dwImageTransformationHandle_t ImageTransformationEngine_ = DW_NULL_HANDLE;
  std::unique_ptr<ImageConverter> imageConverterRgba_;

  dwImageHandle_t imgRgb8_ = DW_NULL_HANDLE;
};
