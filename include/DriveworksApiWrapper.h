// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#pragma once

#include <dw/core/Context.h>
#include <dw/core/VersionCurrent.h>
#include <dw/sensors/Sensors.h>
#include <ros/ros.h>

class DriveworksApiWrapper
{
public:
  /**
   * @brief Constructor, initializes SAL & context handles
   */
  DriveworksApiWrapper();

  /**
   * @brief Destructor, releases SAL & context handles
   */
  virtual ~DriveworksApiWrapper();
  dwContextHandle_t context_handle_ = DW_NULL_HANDLE;
  dwSALHandle_t sal_handle_ = DW_NULL_HANDLE;

private:
  void initialize_sal_handle();
  void initialize_context_handle();
  dwContextParameters sdkParams_ = {};
};
