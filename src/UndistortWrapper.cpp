// Created by Maxandre Ogeret.
// (c) 2021 University of Tartu - Autonomous Driving Lab.

#include "UndistortWrapper.h"

UndistortWrapper::UndistortWrapper(std::shared_ptr<DriveworksApiWrapper> driveworksApiWrapper,
                                   ros::NodeHandle* nodehandle, std::string interface, std::string link)
  : driveworksApiWrapper_(driveworksApiWrapper), nh_(*nodehandle), interface_(interface), link_(link)
{
  loadCalibFile();
  initInModel();
  initOutModel();

  // Init Rectifier
  CHECK_DW_ERROR_ROS(
      dwRectifier_initialize(&rectifier_, cameraModelIn_, cameraModelOut_, driveworksApiWrapper_->context_handle_))

  CHECK_DW_ERROR_ROS(dwRectifier_setHomography(&homography_, rectifier_));
}

void UndistortWrapper::loadCalibFile()
{
  nh_.param<std::string>("calib_dir_path", calib_dir_path_,
                         ros::package::getPath(ros::this_node::getName().substr(1)) + "/calib");
  calib_file_path_ = calib_dir_path_ + "interface" + interface_ + "_link" + link_ + ".yaml";
  ROS_DEBUG_STREAM("Loading calibration file for rectification: " << calib_file_path_);
  config_ = YAML::LoadFile(calib_file_path_);

  if (!config_["distortion_model"] || config_["distortion_model"].as<std::string>() != "plumb_bob")
  {
    ROS_FATAL("Unable to read model, or model is not plumb_bob");
    throw std::runtime_error("Unable to read model from yaml file, or model is not plumb_bob");
  }
}

void UndistortWrapper::initInModel()
{
  pinholeConfIn_.distortion[0] = config_["distortion_coefficients"]["data"][0].as<float32_t>();  // k1
  pinholeConfIn_.distortion[1] = config_["distortion_coefficients"]["data"][1].as<float32_t>();  // k2

  pinholeConfIn_.u0 = config_["camera_matrix"]["data"][2].as<float32_t>();      // cx
  pinholeConfIn_.v0 = config_["camera_matrix"]["data"][5].as<float32_t>();      // cy
  pinholeConfIn_.focalX = config_["camera_matrix"]["data"][0].as<float32_t>();  // fx
  pinholeConfIn_.focalY = config_["camera_matrix"]["data"][4].as<float32_t>();  // fy

  pinholeConfIn_.width = config_["image_width"].as<uint32_t>();
  pinholeConfIn_.height = config_["image_height"].as<uint32_t>();

  CHECK_DW_ERROR_ROS(
      dwCameraModel_initializePinhole(&cameraModelIn_, &pinholeConfIn_, driveworksApiWrapper_->context_handle_))
}

void UndistortWrapper::initOutModel()
{
  pinholeConfOut_.distortion[0] = 0.f;
  pinholeConfOut_.distortion[1] = 0.f;
  pinholeConfOut_.distortion[2] = 0.f;

  pinholeConfOut_.u0 = static_cast<float32_t>(1920.f / 2);
  pinholeConfOut_.v0 = static_cast<float32_t>(1208.f / 2);
  pinholeConfOut_.width = pinholeConfIn_.width;
  pinholeConfOut_.height = pinholeConfIn_.height;

  dwVector2f focalOut = focalFromFOV({ config_["fov"][0].as<float32_t>(), config_["fov"][1].as<float32_t>() },
                                     { pinholeConfOut_.width, pinholeConfOut_.height });
  pinholeConfOut_.focalX = focalOut.x;
  pinholeConfOut_.focalY = focalOut.y;

  CHECK_DW_ERROR_ROS(
      dwCameraModel_initializePinhole(&cameraModelOut_, &pinholeConfOut_, driveworksApiWrapper_->context_handle_))
}

#ifdef VIBRANTE
bool UndistortWrapper::undistortImage(dwImageHandle_t* in_image, dwImageNvMedia* out_image)
{
  dwImageNvMedia* in_image_nvmedia;
  CHECK_DW_ERROR_ROS(dwImage_getNvMedia(&in_image_nvmedia, *in_image))
  CHECK_DW_ERROR_ROS(dwRectifier_warpNvMedia(out_image, in_image_nvmedia, rectifier_))

  return true;
}
#endif

dwVector2f UndistortWrapper::focalFromFOV(const dwVector2f fov, dwVector2ui imageSize)
{
  return dwVector2f{ static_cast<float32_t>(imageSize.x / (2.0 * std::tan(fov.x * M_PI_2 / 180))),
                     static_cast<float32_t>(imageSize.y / (2.0 * std::tan(fov.y * M_PI_2 / 180))) };
}

UndistortWrapper::~UndistortWrapper()
{
  CHECK_DW_ERROR_NOTHROW(dwCameraModel_release(cameraModelIn_));
  CHECK_DW_ERROR_NOTHROW(dwCameraModel_release(cameraModelOut_));
  CHECK_DW_ERROR_NOTHROW(dwRectifier_release(rectifier_))
}