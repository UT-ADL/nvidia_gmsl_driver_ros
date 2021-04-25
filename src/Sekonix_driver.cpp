// Created by Maxandre Ogeret.
// (c) 2021 University of Tartu - Autonomous Driving Lab.

#include "Sekonix_driver.h"
#include <iostream>
#include <ros/ros.h>
#include "signal.h"

Sekonix_driver::Sekonix_driver(ros::NodeHandle *nodehandle) : nh_(*nodehandle) {
  nh_.param<std::string>(
      "config_path", config_file_path_,
      ros::package::getPath(ros::this_node::getName().substr(1)) +
      "/config/ports.yaml");

  // Parse YAML config.
  config_ = YAML::LoadFile(config_file_path_);

  // Create API Wrapper
  driveworksApiWrapper_ = std::make_shared<DriveworksApiWrapper>();
}

bool Sekonix_driver::setup_cameras() {
  for (YAML::const_iterator conn_it = config_.begin(); conn_it != config_.end(); ++conn_it) {
    for (YAML::const_iterator port_it = conn_it->second.begin(); port_it != conn_it->second.end(); ++port_it) {
      for (YAML::const_iterator cam_it = port_it->second.begin(); cam_it != port_it->second.end(); ++cam_it) {
        ROS_DEBUG_STREAM("Connector " << cam_it->first.as<std::string>() << " :  " << cam_it->second["parameters"]["camera-name"].as<std::string>());
        try {
          camera_vector_.push_back(
              std::make_shared<Camera>(driveworksApiWrapper_,
                                       cam_it->second,
                                       port_it->first.as<std::string>(),
                                       cam_it->first.as<std::string>(),
                                       &nh_)
          );
        } catch (const std::runtime_error) {
          return false;
        }
        camera_count++;
      }
    }
  }
  for (auto & camera : camera_vector_) {
    camera->start();
  }
  ROS_INFO_STREAM(camera_vector_.size() << " cameras initialized.");
  return true;
}

bool Sekonix_driver::poll_and_process() {
  for (auto &camera : camera_vector_) {
    try {
      if (camera->poll()) {
        camera->publish();
        camera->clean();
        tries_ = 0;
      } else {
        tries_++;
        if (tries_ > MAX_TRIES_) {
          ROS_FATAL_STREAM("Camera unresponsive");
          return false;
        }
      }
    } catch (const std::runtime_error) {
      return false;
    }
  }
  return true;
}