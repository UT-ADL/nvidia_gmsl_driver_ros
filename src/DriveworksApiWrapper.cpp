// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#include "DriveworksApiWrapper.h"

DriveworksApiWrapper::DriveworksApiWrapper()
{
  initialize_context_handle();
  initialize_sal_handle();
}

void DriveworksApiWrapper::initialize_context_handle()
{
  dwStatus result;
  result = dwInitialize(&context_handle_, DW_VERSION, &sdkParams_);
  if (result != DW_SUCCESS) {
    throw NvidiaGmslDriverRosFatalException("Cannot initialize SAL: " + std::string(dwGetStatusName(result)) + ".");
  }
}

DriveworksApiWrapper::~DriveworksApiWrapper()
{
  dwSAL_release(sal_handle_);
  dwRelease(context_handle_);
}

void DriveworksApiWrapper::initialize_sal_handle()
{
  dwStatus result;
  result = dwSAL_initialize(&sal_handle_, context_handle_);
  if (result != DW_SUCCESS) {
    throw NvidiaGmslDriverRosFatalException("Cannot initialize SAL: " + std::string(dwGetStatusName(result)) + ".");
  }
}
