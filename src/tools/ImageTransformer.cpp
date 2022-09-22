// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#include "tools/ImageTransformer.h"

ImageTransformer::ImageTransformer(DriveworksApiWrapper* driveworksApiWrapper)
  : driveworksApiWrapper_(driveworksApiWrapper)
{
  CHK_DW(dwImage_create(&imgRgb8_, camera_common::get_rgba_pitch_img_prop(), driveworksApiWrapper_->context_handle_));

  dwImageTransformation_initialize(&ImageTransformationEngine_, {}, driveworksApiWrapper_->context_handle_);
  dwImageTransformation_setInterpolationMode(DW_IMAGEPROCESSING_INTERPOLATION_DEFAULT, ImageTransformationEngine_);
}

ImageTransformer::~ImageTransformer()
{
  dwImageTransformation_release(ImageTransformationEngine_);
  dwImage_destroy(imgRgb8_);
}

void ImageTransformer::transform_image(dwImageHandle_t* input, dwImageHandle_t* output)
{
  CHK_DW(dwImage_copyConvert(imgRgb8_, *input, driveworksApiWrapper_->context_handle_));
  CHK_DW(dwImageTransformation_copyFullImage(*output, imgRgb8_, ImageTransformationEngine_));
}