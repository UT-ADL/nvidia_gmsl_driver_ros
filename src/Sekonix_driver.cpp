// Created by Maxandre Ogeret.
// (c) 2021 University of Tartu - Autonomous Driving Lab.

#include "Sekonix_driver.h"

Sekonix_driver::Sekonix_driver(ros::NodeHandle* nodehandle) : nh_(*nodehandle)
{
  nh_.param<std::string>("config_path", config_file_path_,
                         ros::package::getPath(ros::this_node::getName().substr(1)) + "/config/ports.yaml");

  // Parse YAML config.
  config_ = YAML::LoadFile(config_file_path_);

  // Create API Wrapper
  driveworksApiWrapper_ = std::make_shared<DriveworksApiWrapper>();
}

void Sekonix_driver::setup_cameras()
{
  for (YAML::const_iterator conn_it = config_.begin(); conn_it != config_.end(); ++conn_it)
  {
    for (YAML::const_iterator interface_it = conn_it->second.begin(); interface_it != conn_it->second.end(); ++interface_it)
    {
      for (YAML::const_iterator link_it = interface_it->second.begin(); link_it != interface_it->second.end(); ++link_it)
      {
        ROS_DEBUG_STREAM("Link " << link_it->first.as<std::string>() << " :  "
                                      << link_it->second["parameters"]["camera-name"].as<std::string>());
        camera_vector_.push_back(std::make_shared<Camera>(driveworksApiWrapper_, link_it->second,
                                                          interface_it->first.as<std::string>(),
                                                          link_it->first.as<std::string>(), &nh_));
        camera_count++;
      }
    }
  }
  for (auto& camera : camera_vector_)
  {
    camera->start();
  }
  pool_ = std::make_unique<ThreadPool>(camera_count);
  ROS_INFO_STREAM(camera_vector_.size() << " cameras initialized.");
}

void Sekonix_driver::poll_and_process()
{
  for (auto& camera : camera_vector_)
  {
    future_pool_.emplace_back(pool_->enqueue(
        [](const std::shared_ptr<Camera>& camera) -> bool {
          try {
            camera->poll();
            camera->encode();
            camera->publish();
          } catch (SekonixDriverMinorException&) {
            return false;
          }
          return true;
        }, camera));
  }

  all_cameras_valid_ = true;
  for (auto& future : future_pool_) {
    if (!future.get() && all_cameras_valid_) {
      all_cameras_valid_ = false;
      tries_++;
    }
  }
  future_pool_.clear();

  if (tries_ > MAX_TRIES_)
  {
    throw SekonixDriverFatalException("COULDNT REACH CAMERA AFTER " + std::to_string(MAX_TRIES_) + " TRIALS");
  }

  tries_ = all_cameras_valid_ ? 0 : tries_;
}