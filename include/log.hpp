// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#pragma once

#include <dw/core/Logger.h>

#include <string>
#include <iostream>
#include <ros/ros.h>

#include "exceptions/NvidiaGmslDriverRosFatalException.h"

dwLogCallback rosLogWrapper();
