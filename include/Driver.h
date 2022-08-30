// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#pragma once

#include <yaml-cpp/yaml.h>
#include <dw/sensors/Sensors.h>
#include <ros/package.h>
#include <ros/ros.h>

#include "cameras/CameraH264.h"
#include "cameras/CameraJpg.h"
#include <DriveworksApiWrapper.h>
#include "thread_pool.hpp"

#include "exceptions/NvidiaGmslDriverRosFatalException.h"
#include "exceptions/NvidiaGmslDriverRosMinorException.h"

static constexpr size_t MAX_TRIALS_ = 100;

class Driver
{
public:
  /**
   * Constructor
   * @param nodehandle
   */
  explicit Driver(const ros::NodeHandle* nodehandle);

  /**
   * @brief Initializes all parameters and config, handles exception.
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
  std::unique_ptr<thread_pool> pool_;
  std::vector<std::future<bool>> future_pool_;
  size_t trials_ = 0;
  ros::NodeHandle nh_;
  std::string encoder_name_;
  std::string config_file_path_;
  YAML::Node config_;
  std::shared_ptr<DriveworksApiWrapper> driveworksApiWrapper_;
  size_t camera_count = 0;
  std::vector<std::unique_ptr<CameraBase>> camera_vector_;
};
