// Created by Maxandre Ogeret.
// (c) 2021 University of Tartu - Autonomous Driving Lab.

#ifndef SEKONIX_CAMERA_UT_UNDISTORTWRAPPER_H
#define SEKONIX_CAMERA_UT_UNDISTORTWRAPPER_H

#include <cmath>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <dw/rig/Rig.h>
#include <dw/calibration/cameramodel/CameraModel.h>
#include <dw/imageprocessing/geometry/rectifier/Rectifier.h>
#include <dwvisualization/image/FrameCapture.h>

#include "DriveworksApiWrapper.h"
#include "framework/Checks.hpp"

class UndistortWrapper
{
public:
  /**
   * @brief todo
   */
  UndistortWrapper(std::shared_ptr<DriveworksApiWrapper> driveworksApiWrapper, ros::NodeHandle* nodehandle, std::string interface, std::string link);

  /**
   * @brief todo
   */
  void loadCalibFile();

  /**
   * @brief todo
   */
  void initInModel();

  /**
   * @brief todo
   */
  void initOutModel();

#ifdef VIBRANTE
  /**
   * @brief todo
   */
  bool undistortImage(dwImageHandle_t* in_image, dwImageNvMedia* out_image);
#endif

  /**
   * @brief todo
   */
  static dwVector2f focalFromFOV(dwVector2f fov, dwVector2ui imageSize);

  virtual ~UndistortWrapper();

private:
  ros::NodeHandle nh_;
  std::string calib_dir_path_;
  std::string calib_file_path_;
  std::string interface_;
  std::string link_;
  YAML::Node config_;

  dwRectifierHandle_t rectifier_ = DW_NULL_HANDLE;
  dwCameraModelHandle_t cameraModelIn_ = DW_NULL_HANDLE;
  dwCameraModelHandle_t cameraModelOut_ = DW_NULL_HANDLE;

  dwMatrix3f homography_ = DW_IDENTITY_MATRIX3F;

  std::shared_ptr<DriveworksApiWrapper> driveworksApiWrapper_;
  dwPinholeCameraConfig pinholeConfOut_ = {};
  dwPinholeCameraConfig pinholeConfIn_ = {};
};

#endif  // SEKONIX_CAMERA_UT_UNDISTORTWRAPPER_H
