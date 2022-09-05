// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#include "log.hpp"

dwLogCallback ros_log_wrapper()
{
  return [](dwContextHandle_t, dwLoggerVerbosity level, const char* msg) {
    if (strchr(msg, '[') && strchr(msg, ']') && strlen(msg) == 22) {
      // Removing redundant date messages.
      return;
    }

    std::string to_print(msg);
    to_print.erase(std::remove(to_print.begin(), to_print.end(), '\n'), to_print.cend());

    switch (level) {
      case DW_LOG_SILENT:
      case DW_LOG_VERBOSE:
      case DW_LOG_DEBUG:
        ROS_INFO_STREAM(to_print);
        break;
      case DW_LOG_WARN:
        ROS_WARN_STREAM(to_print);
        break;
      case DW_LOG_ERROR:
        ROS_ERROR_STREAM(to_print);
        break;
      default:
        throw NvidiaGmslDriverRosFatalException("Verbosity level " + std::to_string(level) + "unknown.");
    }
  };
}
