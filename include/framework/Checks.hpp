// Copyright (c) 2015-2016 NVIDIA Corporation. All rights reserved.
// Edited by Maxandre Ogeret - 2021 - University of Tartu - Autonomous Driving
// Lab.

#ifndef SAMPLES_COMMON_CHECKS_HPP_
#define SAMPLES_COMMON_CHECKS_HPP_

#include <dwvisualization/gl/GL.h>

#include <cuda_runtime.h>

#include <iostream>
#include <stdexcept>
#include <string>

#include <time.h>

#include <ros/ros.h>
#include <nvmedia_core.h>
#include "exceptions/SekonixDriverMinorException.h"
#include "exceptions/SekonixDriverFatalException.h"

//------------------------------------------------------------------------------
// Functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// print GL error with location
inline void getGLError(int line, const char* file, const char* function)
{
  GLenum error = glGetError();
  if (error != GL_NO_ERROR) {
    std::cerr << file << " in function " << function << " in line " << line << ": glError: 0x" << std::hex << error
              << std::dec << std::endl;
    exit(1);
  }
}

#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __FUNCTION__
#endif

// macro to easily check for GL errors
#define CHECK_GL_ERROR() getGLError(__LINE__, __FILE__, __PRETTY_FUNCTION__);

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
      throw SekonixDriverFatalException(std::string(buf) + std::string("DW Error ") + dwGetStatusName(result) +        \
                                        std::string(" executing DW function:\n " #x) +                                 \
                                        std::string("\n at " __FILE__ ":") + std::to_string(__LINE__));                \
    }                                                                                                                  \
  };

#define CHECK_DW_ERROR_ROS_MINOR(x)                                                                                    \
  {                                                                                                                    \
    dwStatus result = x;                                                                                               \
    if (result != DW_SUCCESS) {                                                                                        \
      char buf[80];                                                                                                    \
      getDateString(buf, 80);                                                                                          \
      throw SekonixDriverMinorException(std::string(buf) + std::string("DW Error ") + dwGetStatusName(result) +        \
                                        std::string(" executing DW function:\n " #x) +                                 \
                                        std::string("\n at " __FILE__ ":") + std::to_string(__LINE__));                \
    }                                                                                                                  \
  };

#define CHECK_NVMEDIA_ERROR_ROS_FATAL(x)                                                                               \
  {                                                                                                                    \
    NvMediaStatus result = x;                                                                                          \
    if (result != NVMEDIA_STATUS_OK) {                                                                                 \
      char buf[80];                                                                                                    \
      getDateString(buf, 80);                                                                                          \
      throw SekonixDriverFatalException(std::string(buf) + std::string(" NVMEDIA Error id:") + std::to_string(x));     \
    }                                                                                                                  \
  };

#endif  // SAMPLES_COMMON_CHECKS_HPP_
