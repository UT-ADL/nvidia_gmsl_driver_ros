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

bool Sekonix_driver::setup_cameras()
{
  for (YAML::const_iterator conn_it = config_.begin(); conn_it != config_.end(); ++conn_it)
  {
    for (YAML::const_iterator interface_it = conn_it->second.begin(); interface_it != conn_it->second.end(); ++interface_it)
    {
      for (YAML::const_iterator link_it = interface_it->second.begin(); link_it != interface_it->second.end(); ++link_it)
      {
        ROS_DEBUG_STREAM("Link " << link_it->first.as<std::string>() << " :  "
                                      << link_it->second["parameters"]["camera-name"].as<std::string>());
        try
        {
          camera_vector_.push_back(std::make_shared<Camera>(driveworksApiWrapper_, link_it->second,
                                                            interface_it->first.as<std::string>(),
                                                            link_it->first.as<std::string>(), &nh_));
        }
        catch (const std::runtime_error)
        {
          return false;
        }
        camera_count++;
      }
    }
  }
  for (auto& camera : camera_vector_)
  {
    camera->start();
  }
  ROS_INFO_STREAM(camera_vector_.size() << " cameras initialized.");
  return true;
}

bool Sekonix_driver::poll_and_process()
{
  for (auto& camera : camera_vector_)
  {
    try
    {
      if (camera->poll())
      {
        camera->publish();
        camera->clean();
        tries_ = 0;
      }
      else
      {
        tries_++;
        if (tries_ > MAX_TRIES_)
        {
          ROS_FATAL_STREAM("COULDNT REACH CAMERA AFTER " << MAX_TRIES_ << " TRIALS");
          return false;
        }
      }
    }
    catch (const std::runtime_error)
    {
      ROS_FATAL_STREAM("FATAL ERROR WHILE POLLING AND PROCESSING");
      return false;
    }
  }
  return true;
}