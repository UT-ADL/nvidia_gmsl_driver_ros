// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#pragma once

#include <stdexcept>
#include <string>
#include <memory>

/**
 * @class NvidiaGmslDriverRosMinorException
 * @brief Thrown when the nvidia_gmsl_driver_ros encounters a recoverable error
 */
class NvidiaGmslDriverRosMinorException : public std::runtime_error
{
public:
  explicit NvidiaGmslDriverRosMinorException(const std::string& description) : std::runtime_error(description) {}
};
