//
// Created by Maxandre Ogeret.
//

#include "NvMediaH264Encoder.h"
NvMediaH264Encoder::NvMediaH264Encoder(NvMediaSurfaceType surfaceType): surfaceType_(surfaceType)
{
  NvMediaIEPGetVersion(nvMediaVersion_);

  // Device
  nvMediaDevice_ = NvMediaDeviceCreate();

  // Params todo read params from yaml file
  initParams_.encodeWidth = 1920;
  initParams_.encodeHeight = 1208;
  initParams_.frameRateNum = 30;
  initParams_.frameRateDen = 1;
  initParams_.profile = NVMEDIA_ENCODE_PROFILE_AUTOSELECT;
  initParams_.level = 2;

  picParams_.pictureType = NVMEDIA_ENCODE_PIC_TYPE_AUTOSELECT;
  picParams_.frameRateNum = initParams_.frameRateNum;
  picParams_.frameRateDen = initParams_.frameRateDen;
  picParams_.nextBFrames = 0;

  rect_->x0 = 0;
  rect_->x1 = 1980;
  rect_->y0 = 0;
  rect_->y1 = 1208;

  encoder_ = NvMediaIEPCreate(nvMediaDevice_, NVMEDIA_IMAGE_ENCODE_H264, &initParams_, surfaceType_, -1, 2, NVMEDIA_ENCODER_INSTANCE_0);
  NvMediaIEPSetConfiguration(encoder_, &config_);
}

NvMediaH264Encoder::~NvMediaH264Encoder()
{
  NvMediaIEPDestroy(encoder_);
}

void NvMediaH264Encoder::feed_frame(NvMediaImage* inNvMediaImage)
{
//  CHECK_NVMEDIA_ERROR_ROS_FATAL(NvMediaIEPImageRegister(encoder_, inNvMediaImage, NVMEDIA_ACCESS_MODE_READ)); todo: implement
  CHECK_NVMEDIA_ERROR_ROS_FATAL(NvMediaIEPFeedFrame(encoder_, inNvMediaImage, rect_, &picParams_, NVMEDIA_ENCODER_INSTANCE_0));
}
bool NvMediaH264Encoder::bits_available()
{
  status_ = NvMediaIEPBitsAvailable(encoder_, &numBytesAvailable_, NVMEDIA_ENCODE_BLOCKING_TYPE_NEVER, 0);

  if (status_ == NVMEDIA_STATUS_OK) {
    return true;
  } else if (status_ == NVMEDIA_STATUS_PENDING or status_ == NVMEDIA_STATUS_NONE_PENDING) {
    return false;
  } else {
    throw SekonixDriverFatalException("NvMedia H264 Encoder error #"+std::to_string(status_));
  }
}
void NvMediaH264Encoder::get_bits()
{
  NvMediaIEPGetBitsEx(encoder_, &numBytesAvailable_, 0, &buffer_, nullptr);
}
