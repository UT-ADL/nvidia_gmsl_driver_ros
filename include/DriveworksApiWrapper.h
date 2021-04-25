// Created by Maxandre Ogeret.
// (c) 2021 University of Tartu - Autonomous Driving Lab.

#ifndef SEKONIX_CAMERA_UT_REWRITE_DRIVEWORKSAPIWRAPPER_H
#define SEKONIX_CAMERA_UT_REWRITE_DRIVEWORKSAPIWRAPPER_H

#include <ros/ros.h>
#include <dw/core/Context.h>
#include <dw/core/VersionCurrent.h>
#include <dw/sensors/Sensors.h>

class DriveworksApiWrapper {

public:
  DriveworksApiWrapper();
  virtual ~DriveworksApiWrapper();
  dwContextHandle_t context_handle_ = DW_NULL_HANDLE;
  dwSALHandle_t sal_handle_ = DW_NULL_HANDLE;

private:
  void InitializeSalHandle();
  void InitializeContextHandle();

  dwContextParameters sdkParams_ = {};
};

#endif // SEKONIX_CAMERA_UT_REWRITE_DRIVEWORKSAPIWRAPPER_H
