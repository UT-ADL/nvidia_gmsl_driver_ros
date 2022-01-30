// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#include "log.hpp"

dwLogCallback colorLogger()
{
  return [](dwContextHandle_t, dwLoggerVerbosity level, const char* msg) {
    switch (level) {
      case DW_LOG_SILENT:
      case DW_LOG_VERBOSE:
      case DW_LOG_DEBUG:
        std::cout << ANSI_STD << msg << ANSI_STD;
        break;
      case DW_LOG_WARN:
        std::cout << ANSI_YEL << msg << ANSI_STD;
        break;
      case DW_LOG_ERROR:
        std::cerr << ANSI_RED << msg << ANSI_STD;
        break;
    }
  };
}
