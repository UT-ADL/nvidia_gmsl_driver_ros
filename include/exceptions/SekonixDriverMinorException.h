// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#ifndef SEKONIX_CAMERA_UT_SEKONIXDRIVERMINOREXCEPTION_H
#define SEKONIX_CAMERA_UT_SEKONIXDRIVERMINOREXCEPTION_H

#include <stdexcept>
#include <string>
#include <memory>

/**
 * @class SekonixDriverMinorException
 * @brief Thrown when the Sekonix camera UT driver encounters a recoverable error
 */
class SekonixDriverMinorException : public std::runtime_error
{
public:
  explicit SekonixDriverMinorException(const std::string& description) : std::runtime_error(description) {}
};

#endif  // SEKONIX_CAMERA_UT_SEKONIXDRIVERMINOREXCEPTION_H
