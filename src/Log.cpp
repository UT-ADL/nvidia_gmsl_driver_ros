// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#include "Log.hpp"

dwLogCallback colorLogger()
{
  return [](dwContextHandle_t, dwLoggerVerbosity level, const char* msg) {
    switch (level) {
      case DW_LOG_SILENT:
      case DW_LOG_VERBOSE:
      case DW_LOG_DEBUG:
        break;
      case DW_LOG_WARN:
        std::cout << ANSI_YEL << msg << ANSI_STD << std::endl;
        break;
      case DW_LOG_ERROR:
        std::cerr << ANSI_RED << msg << ANSI_STD << std::endl;
        break;
    }
  };
}