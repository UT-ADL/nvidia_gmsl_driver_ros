// Created by Maxandre Ogeret.
// (c) 2021 University of Tartu - Autonomous Driving Lab.

#include "cameras/CameraBase.h"

CameraBase::CameraBase(const std::shared_ptr<DriveworksApiWrapper> driveworksApiWrapper, const YAML::Node& config,
                       const std::string interface, const std::string link, ros::NodeHandle* nodehandle)
  : driveworksApiWrapper_(driveworksApiWrapper), config_(config), interface_(interface), link_(link), nh_(*nodehandle)
{
  nh_.param<int>("framerate", framerate_, 30);

  // Read params from yaml
  for (YAML::const_iterator param_it = config_["parameters"].begin(); param_it != config_["parameters"].end();
       ++param_it) {
    params_ += param_it->first.as<std::string>() + "=" + param_it->second.as<std::string>() + ",";
  }

  params_ += "interface=csi-" + interface_ + ",";
  params_ += "link=" + link_;
  sensorParams_.parameters = params_.c_str();
  sensorParams_.protocol = "camera.gmsl";
  CHECK_DW_ERROR_ROS(dwSAL_createSensor(&sensorHandle_, sensorParams_, driveworksApiWrapper_->sal_handle_))

  // ROS
  frame_ = "interface" + interface_ + "_link" + link_;

  // Calibration
  nh_.param<std::string>("calib_dir_path", calibDirPath_,
                         ros::package::getPath(ros::this_node::getName().substr(1)) + "/calib/");
  camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
      ros::NodeHandle(config_["topic"].as<std::string>()), frame_);
  cam_info_file_ << "file://" << calibDirPath_ << frame_ << ".yaml";
  if (camera_info_manager_->validateURL(cam_info_file_.str()) &&
      camera_info_manager_->loadCameraInfo(cam_info_file_.str())) {
    //        camera_info_ = camera_info_manager_->getCameraInfo();
  }

  ROS_DEBUG_STREAM("CameraH264 on interface : " << interface_ << ", link : " << link_ << " initialized successfully!");
}

CameraBase::~CameraBase()
{
  dwSAL_releaseSensor(sensorHandle_);
  ROS_DEBUG("CAMERA RELEASED !");
}

void CameraBase::start()
{
  // Start the sensor
  CHECK_DW_ERROR_ROS(dwSensor_start(sensorHandle_))
}

bool CameraBase::get_last_frame()
{
  dwStatus status_old = DW_NOT_READY;

  while (true) {
    status_ = dwSensorCamera_readFrameNew(&cameraFrameHandle_, 0, sensorHandle_);

    if (status_ == DW_NOT_READY) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      return false;
    }

    if (status_ == DW_TIME_OUT && status_old == DW_SUCCESS) {
      return true;
    }

    status_old = status_;
  }
}