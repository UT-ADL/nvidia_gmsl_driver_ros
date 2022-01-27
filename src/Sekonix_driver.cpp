// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#include "Sekonix_driver.h"

Sekonix_driver::Sekonix_driver(ros::NodeHandle* nodehandle) : nh_(*nodehandle)
{
  nh_.param<std::string>("encoder", encoder_name_, CameraJpg::ENCODER_TYPE_);

  nh_.param<std::string>("config_path", config_file_path_,
                         ros::package::getPath(ros::this_node::getName().substr(1)) + "/config/ports.yaml");

  ROS_INFO_STREAM("encoder: " << encoder_name_);
  ROS_INFO_STREAM("config_path: " << config_file_path_);

  // Parse YAML config.
  config_ = YAML::LoadFile(config_file_path_);

  // Create API Wrapper
  driveworksApiWrapper_ = std::make_unique<DriveworksApiWrapper>();
}

void Sekonix_driver::setup_cameras()
{
  for (YAML::const_iterator conn_it = config_.begin(); conn_it != config_.end(); ++conn_it) {
    for (YAML::const_iterator interface_it = conn_it->second.begin(); interface_it != conn_it->second.end();
         ++interface_it) {
      for (YAML::const_iterator link_it = interface_it->second.begin(); link_it != interface_it->second.end();
           ++link_it) {
        ROS_DEBUG_STREAM("Link " << link_it->first.as<std::string>() << " :  "
                                 << link_it->second["parameters"]["camera-name"].as<std::string>());
        create_camera(link_it->second, interface_it->first.as<std::string>(), link_it->first.as<std::string>());
        camera_count++;
      }
    }
  }
  for (auto& camera : camera_vector_) {
    camera->start();
  }
  pool_ = std::make_unique<thread_pool>(camera_count);
  ROS_INFO_STREAM(camera_vector_.size() << " cameras initialized.");

  if (camera_vector_.empty()) {
    throw SekonixDriverFatalException("No cameras were initialized.");
  }
}

void Sekonix_driver::create_camera(const YAML::Node& config, const std::string& interface, const std::string& link)
{
  if (encoder_name_ == CameraH264::ENCODER_TYPE_) {
    camera_vector_.emplace_back(
        std::make_unique<CameraH264>(driveworksApiWrapper_.get(), config, interface, link, &nh_));
  } else if (encoder_name_ == CameraJpg::ENCODER_TYPE_) {
    camera_vector_.emplace_back(
        std::make_unique<CameraJpg>(driveworksApiWrapper_.get(), config, interface, link, &nh_));
  }
}

void Sekonix_driver::poll_and_process()
{
  for (auto& camera : camera_vector_) {
    future_pool_.emplace_back(pool_->submit(
        [](CameraBase* camera) -> bool {
          try {
            camera->run_pipeline();
          }
          catch (SekonixDriverMinorException&) {
            return false;
          }
          return true;
        },
        camera.get()));
  }

  all_cameras_valid_ = true;
  for (auto& future : future_pool_) {
    if (!future.get() && all_cameras_valid_) {
      all_cameras_valid_ = false;
      tries_++;
    }
  }
  future_pool_.clear();

  if (tries_ > MAX_TRIES_) {
    throw SekonixDriverFatalException("COULDN'T REACH CAMERA AFTER " + std::to_string(MAX_TRIES_) + " TRIALS");
  }

  tries_ = all_cameras_valid_ ? 0 : tries_;
}