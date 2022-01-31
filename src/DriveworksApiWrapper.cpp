// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#include "DriveworksApiWrapper.h"

DriveworksApiWrapper::DriveworksApiWrapper()
{
  InitializeContextHandle();
  InitializeSalHandle();
}

void DriveworksApiWrapper::InitializeContextHandle()
{
  dwInitialize(&context_handle_, DW_VERSION, &sdkParams_);
}

DriveworksApiWrapper::~DriveworksApiWrapper()
{
  dwSAL_release(sal_handle_);
  dwRelease(context_handle_);
  ROS_DEBUG("DRIVEWORKS API HANDLES RELEASED !");
}

void DriveworksApiWrapper::InitializeSalHandle()
{
  dwStatus result;
  result = dwSAL_initialize(&sal_handle_, context_handle_);
  if (result != DW_SUCCESS) {
    ROS_ERROR_STREAM("Cannot initialize SAL: " << dwGetStatusName(result));
    exit(1);
  }
}
