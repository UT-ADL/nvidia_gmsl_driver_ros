// Created by Maxandre Ogeret.
// (c) 2021 University of Tartu - Autonomous Driving Lab.

#ifndef SEKONIX_CAMERA_UT_SEKONIXDRIVERFATALEXCEPTION_H
#define SEKONIX_CAMERA_UT_SEKONIXDRIVERFATALEXCEPTION_H

#include <stdexcept>
#include <string>
#include <memory>

/**
 * @class SekonixDriverFatalException
 * @brief Thrown when the Sekonix camera UT driver encounters a fatal error
 */
class SekonixDriverFatalException : public std::runtime_error
{
public:
  explicit SekonixDriverFatalException(const std::string& description) : std::runtime_error(description) {}
};

#endif  // SEKONIX_CAMERA_UT_SEKONIXDRIVERFATALEXCEPTION_H
