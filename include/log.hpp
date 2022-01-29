// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#pragma once

#include <dw/core/Logger.h>

#include <string>
#include <iostream>

const std::string ANSI_STD = "\033[0m";
const std::string ANSI_RED = "\033[0;31m";
const std::string ANSI_YEL = "\033[0;33m";

dwLogCallback colorLogger();
