// Copyright (c) 2015-2016 NVIDIA Corporation. All rights reserved.
// Edited by Maxandre Ogeret - 2022 - University of Tartu - Autonomous Driving
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

inline const std::unordered_map<int, std::string> NVMEDIA_ERROR_TO_STRING = {
  { 0, "NVMEDIA_STATUS_OK" },
  { 1, "NVMEDIA_STATUS_BAD_PARAMETER" },
  { 2, "NVMEDIA_STATUS_PENDING" },
  { 3, "NVMEDIA_STATUS_TIMED_OUT" },
  { 4, "NVMEDIA_STATUS_OUT_OF_MEMORY" },
  { 5, "NVMEDIA_STATUS_NOT_INITIALIZED" },
  { 6, "NVMEDIA_STATUS_NOT_SUPPORTED" },
  { 7, "NVMEDIA_STATUS_ERROR" },
  { 8, "NVMEDIA_STATUS_NONE_PENDING" },
  { 9, "NVMEDIA_STATUS_INSUFFICIENT_BUFFERING" },
  { 10, "NVMEDIA_STATUS_INVALID_SIZE" },
  { 11, "NVMEDIA_STATUS_INCOMPATIBLE_VERSION" },
  { 13, "NVMEDIA_STATUS_UNDEFINED_STATE" },
  { 14, "NVMEDIA_STATUS_PFSD_ERROR" }
};

inline void getDateString(char* buf, size_t length)
{
  time_t now = ::time(nullptr);
  struct tm const* calendar = localtime(&now);
  strftime(buf, length, "[%Y-%m-%d %X] ", calendar);
}

#define CHK_DW(x)                                                                                                      \
  do {                                                                                                                 \
    dwStatus result = x;                                                                                               \
    if (result != DW_SUCCESS) {                                                                                        \
      char buf[80];                                                                                                    \
      getDateString(buf, 80);                                                                                          \
      throw NvidiaGmslDriverRosFatalException(std::string(buf) + std::string("DW Error ") + dwGetStatusName(result) +  \
                                              std::string(" executing DW function:\n " #x) +                           \
                                              std::string("\n at " __FILE__ ":") + std::to_string(__LINE__));          \
    }                                                                                                                  \
  } while (0)

#define CHK_DW_MINOR(x)                                                                                                \
  do {                                                                                                                 \
    dwStatus result = x;                                                                                               \
    if (result != DW_SUCCESS) {                                                                                        \
      char buf[80];                                                                                                    \
      getDateString(buf, 80);                                                                                          \
      throw NvidiaGmslDriverRosMinorException(std::string(buf) + std::string("DW Error ") + dwGetStatusName(result) +  \
                                              std::string(" executing DW function:\n " #x) +                           \
                                              std::string("\n at " __FILE__ ":") + std::to_string(__LINE__));          \
    }                                                                                                                  \
  } while (0)

#define CHK_NVM(x)                                                                                                     \
  do {                                                                                                                 \
    NvMediaStatus result = x;                                                                                          \
    if (result != NVMEDIA_STATUS_OK) {                                                                                 \
      char buf[80];                                                                                                    \
      getDateString(buf, 80);                                                                                          \
      throw NvidiaGmslDriverRosFatalException(std::string(buf) + std::string(" NVMEDIA Error id: ") +                  \
                                              std::to_string(x) + " : " + NVMEDIA_ERROR_TO_STRING.at(x) + ".");        \
    }                                                                                                                  \
  } while (0)
