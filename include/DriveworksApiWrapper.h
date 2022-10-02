// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#pragma once

#include <dw/core/Context.h>
#include <dw/core/VersionCurrent.h>
#include <dw/sensors/Sensors.h>
#include <ros/ros.h>

#include "exceptions/NvidiaGmslDriverRosFatalException.h"

/**
 * @brief Wrapper around Nvidia Driveworks API. Its main purpose is to manage SAL and context handles.
 */
class DriveworksApiWrapper
{
public:
  /**
   * @brief Constructor, initializes SAL & context handles.
   */
  DriveworksApiWrapper();

  /**
   * @brief Destructor, releases SAL & context handles.
   */
  virtual ~DriveworksApiWrapper();
  dwContextHandle_t context_handle_ = DW_NULL_HANDLE;
  dwSALHandle_t sal_handle_ = DW_NULL_HANDLE;

private:
  /**
   * @brief Initializes SAL handle.
   * @throws NvidiaGmslDriverRosFatalException
   */
  void initialize_sal_handle();

  /**
   * @brief Initializes context handle.
   * @throws NvidiaGmslDriverRosFatalException
   */
  void initialize_context_handle();
  dwContextParameters sdkParams_ = {};
};
