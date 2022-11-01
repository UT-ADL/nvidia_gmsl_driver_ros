// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#pragma once

#include <memory>
#include <stdexcept>
#include <string>

/**
 * @brief Thrown when the nvidia_gmsl_driver_ros encounters a recoverable error
 */
class NvidiaGmslDriverRosMinorException : public std::runtime_error
{
public:
  using runtime_error::runtime_error;
};
