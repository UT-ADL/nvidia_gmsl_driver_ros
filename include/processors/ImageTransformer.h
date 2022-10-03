// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#pragma once

#include <dw/image/Image.h>
#include <dw/imageprocessing/geometry/imagetransformation/ImageTransformation.h>
#include <dw/sensors/camera/Camera.h>

#include "DriveworksApiWrapper.h"
#include "cameras/CameraCommon.h"
#include "framework/Checks.hpp"

/**
 * @brief Wrapper class around the Nvidia DriveWorks Image Transformation Interface.
 * The Image Transformation Interface API is used to transform (resize) images.
 * @see https://docs.nvidia.com/drive/driveworks-3.5/group__imagetransformation__group.html
 */
class ImageTransformer
{
public:
  ImageTransformer(DriveworksApiWrapper* driveworksApiWrapper);

  /**
   * @brief Transforms (resizes) the input image. The transformed image is copied to output.
   */
  void transform_image(dwImageHandle_t* input, dwImageHandle_t* output);

  /**
   * @brief Destructor.
   */
  virtual ~ImageTransformer();

private:
  DriveworksApiWrapper* driveworksApiWrapper_;
  dwImageTransformationHandle_t ImageTransformationEngine_ = DW_NULL_HANDLE;
  dwImageHandle_t imgRgb8_ = DW_NULL_HANDLE;
};
