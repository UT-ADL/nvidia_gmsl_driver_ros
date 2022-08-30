// Copyright (c) 2015-2016 NVIDIA Corporation. All rights reserved.
// Edited by Maxandre Ogeret - 2021 - University of Tartu - Autonomous Driving
// Lab.

#pragma once

#include <dwvisualization/gl/GL.h>

#include <cuda_runtime.h>

#include <iostream>
#include <stdexcept>
#include <string>

#include <time.h>

#include <ros/ros.h>
#include <nvmedia_core.h>
#include "exceptions/NvidiaGmslDriverRosMinorException.h"
#include "exceptions/NvidiaGmslDriverRosFatalException.h"

inline void getDateString(char* buf, size_t length)
{
  time_t now = ::time(0);
  struct tm* calendar = localtime(&now);
  strftime(buf, length, "[%Y-%m-%d %X] ", calendar);
}

#define CHECK_DW_ERROR_ROS(x)                                                                                          \
  {                                                                                                                    \
    dwStatus result = x;                                                                                               \
    if (result != DW_SUCCESS) {                                                                                        \
      char buf[80];                                                                                                    \
      getDateString(buf, 80);                                                                                          \
      std::cout << std::string(buf) + std::string("DW Error ") + dwGetStatusName(result) +                             \
                       std::string(" executing DW function:\n " #x) + std::string("\n at " __FILE__ ":") +             \
                       std::to_string(__LINE__);                                                                       \
      throw NvidiaGmslDriverRosFatalException(std::string(buf) + std::string("DW Error ") + dwGetStatusName(result) +  \
                                              std::string(" executing DW function:\n " #x) +                           \
                                              std::string("\n at " __FILE__ ":") + std::to_string(__LINE__));          \
    }                                                                                                                  \
  };

#define CHECK_DW_ERROR_ROS_MINOR(x)                                                                                    \
  {                                                                                                                    \
    dwStatus result = x;                                                                                               \
    if (result != DW_SUCCESS) {                                                                                        \
      char buf[80];                                                                                                    \
      getDateString(buf, 80);                                                                                          \
      throw NvidiaGmslDriverRosMinorException(std::string(buf) + std::string("DW Error ") + dwGetStatusName(result) +  \
                                              std::string(" executing DW function:\n " #x) +                           \
                                              std::string("\n at " __FILE__ ":") + std::to_string(__LINE__));          \
    }                                                                                                                  \
  };

#define CHECK_NVMEDIA_ERROR_ROS(x)                                                                                     \
  {                                                                                                                    \
    NvMediaStatus result = x;                                                                                          \
    if (result != NVMEDIA_STATUS_OK) {                                                                                 \
      char buf[80];                                                                                                    \
      getDateString(buf, 80);                                                                                          \
      throw NvidiaGmslDriverRosFatalException(std::string(buf) + std::string(" NVMEDIA Error id:") +                   \
                                              std::to_string(x));                                                      \
    }                                                                                                                  \
  };
