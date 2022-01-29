// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#pragma once

#include <stdexcept>
#include <string>
#include <memory>

/**
 * @class NvidiaGmslDriverRosFatalException
 * @brief Thrown when the nvidia_gmsl_driver_ros encounters a fatal error
 */
class NvidiaGmslDriverRosFatalException : public std::runtime_error
{
public:
  explicit NvidiaGmslDriverRosFatalException(const std::string& description) : std::runtime_error(description) {}
};
