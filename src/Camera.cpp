// Created by Maxandre Ogeret.
// (c) 2021 University of Tartu - Autonomous Driving Lab.

#include "Camera.h"

#include <utility>

Camera::Camera(const std::shared_ptr<DriveworksApiWrapper> driveworksApiWrapper, const YAML::Node& config,
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
  pub_h264 = nh_.advertise<sekonix_camera_ut::H264Packet>(config_["topic"].as<std::string>() + "/image/h264", 0);

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

  serializerUserData_.publisher = &pub_h264;
  serializerUserData_.timestamp = &timestamp_;
  serializerUserData_.frame_id = &frame_;

  // Serializer
  start_serializer();

  ROS_DEBUG_STREAM("Camera on interface : " << interface_ << ", link : " << link_ << " initialized successfully!");
}

Camera::~Camera()
{
  dwSAL_releaseSensor(sensorHandle_);
  ROS_DEBUG("CAMERA RELEASED !");

  if (camera_serializer_) {
    dwSensorSerializer_stop(camera_serializer_);
    dwSensorSerializer_release(camera_serializer_);
  }
}

void Camera::start_serializer()
{
  serializer_config_string_ =
      "format=h264"
      ",bitrate=8000000"
      ",type=user";
  serializer_config_string_ += ",framerate=" + std::to_string(framerate_);

  dwSerializerParams serializerParams;

  serializerParams.parameters = serializer_config_string_.c_str();
  serializerParams.userData = &serializerUserData_;

  serializerParams.onData = [](const uint8_t* data, size_t size, void* userData) -> void {
    auto* userDataCast = static_cast<serializer_user_data_t_*>(userData);
    sekonix_camera_ut::H264Packet msg;
    msg.data.assign(data, data + size);
    msg.header.stamp = ros::Time((static_cast<double>(*userDataCast->timestamp) * 10e-7));
    msg.header.frame_id = *userDataCast->frame_id;
    userDataCast->publisher->publish(msg);
  };

  CHECK_DW_ERROR_ROS(dwSensorSerializer_initialize(&camera_serializer_, &serializerParams, sensorHandle_))
  CHECK_DW_ERROR_ROS(dwSensorSerializer_start(camera_serializer_))
  ROS_DEBUG("INITIALIZED SERIALIZER");
}

void Camera::start()
{
  // Start the sensor
  CHECK_DW_ERROR_ROS(dwSensor_start(sensorHandle_))
}

bool Camera::get_last_frame()
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

void Camera::poll()
{
  if (!get_last_frame()) {
    throw SekonixDriverMinorException("Unable to get frame");
  }

  CHECK_DW_ERROR_ROS(
      dwSensorCamera_getImage(&imageHandleOriginal_, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, cameraFrameHandle_))
  CHECK_DW_ERROR_ROS(dwImage_getTimestamp(&timestamp_, imageHandleOriginal_))
}

void Camera::encode()
{
  CHECK_DW_ERROR_ROS(dwSensorSerializer_serializeCameraFrameAsync(cameraFrameHandle_, camera_serializer_))
  CHECK_DW_ERROR_ROS(dwSensorCamera_returnFrame(&cameraFrameHandle_))
}