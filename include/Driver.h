// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#pragma once

#include <DriveworksApiWrapper.h>
#include <dw/sensors/Sensors.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "cameras/CameraH264.h"
#include "cameras/CameraH265.h"
#include "cameras/CameraJpg.h"
#include "cameras/CameraVp9.h"
#include "exceptions/NvidiaGmslDriverRosFatalException.h"
#include "exceptions/NvidiaGmslDriverRosMinorException.h"
#include "framework/thread_pool.hpp"

static constexpr size_t MAX_TRIALS = 100;

/**
 * @brief Main class of the driver. Creates and runs camera pipelines, handles exception.
 */
class Driver
{
public:
  /**
   * Constructor.
   * @param nodehandle
   */
  explicit Driver(const ros::NodeHandle* nodehandle);

  /**
   * @brief Loads camera definition from config and parameters then calls create_camera(). Handles exceptions.
   * @throws NvidiaGmslDriverRosFatalException
   */
  void setup_cameras();

  /**
   * @brief Creates one camera.
   * @param config
   * @param interface
   * @param link
   */
  void create_camera(const YAML::Node& config, const std::string& interface, const std::string& link);

  /**
   * @brief Runs the camera pipelines in a loop, one thread per camera.
   * @throws NvidiaGmslDriverRosFatalException
   * @throws NvidiaGmslDriverRosMinorException
   */
  void run();

private:
  bool all_cameras_valid_;
  std::unique_ptr<BS::thread_pool> pool_;
  std::vector<std::future<bool>> future_pool_;
  size_t trials_ = 0;
  ros::NodeHandle nh_;
  std::string encoder_name_;
  std::string config_file_path_;
  YAML::Node config_;
  std::unique_ptr<DriveworksApiWrapper> driveworksApiWrapper_;
  size_t camera_count_ = 0;
  std::vector<std::unique_ptr<CameraBase>> camera_vector_;
};
