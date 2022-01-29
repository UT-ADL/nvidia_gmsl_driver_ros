// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#ifndef SEKONIX_CAMERA_UT_LOG_H
#define SEKONIX_CAMERA_UT_LOG_H

#include <dw/core/Logger.h>

#include <string>
#include <iostream>

const std::string ANSI_STD = "\033[0m";
const std::string ANSI_RED = "\033[0;31m";
const std::string ANSI_YEL = "\033[0;33m";

dwLogCallback colorLogger();

#endif  // SEKONIX_CAMERA_UT_LOG_H
