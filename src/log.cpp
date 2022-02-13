// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#include "log.hpp"

dwLogCallback rosLogWrapper()
{
  return [](dwContextHandle_t, dwLoggerVerbosity level, const char* msg) {
    switch (level) {
      case DW_LOG_SILENT:
      case DW_LOG_VERBOSE:
      case DW_LOG_DEBUG:
        ROS_INFO_STREAM(msg);
        break;
      case DW_LOG_WARN:
        ROS_WARN_STREAM(msg);
        break;
      case DW_LOG_ERROR:
        ROS_ERROR_STREAM(msg);
        break;
      default:
        throw NvidiaGmslDriverRosFatalException("Verbosity level " + std::to_string(level) + "unknown.");
    }
  };
}
