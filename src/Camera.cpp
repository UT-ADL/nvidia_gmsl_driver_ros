// Created by Maxandre Ogeret.
// (c) 2021 University of Tartu - Autonomous Driving Lab.

#include "Camera.h"

Camera::Camera(std::shared_ptr<DriveworksApiWrapper> driveworksApiWrapper, const YAML::Node config,
               const std::string interface, const std::string link, ros::NodeHandle* nodehandle)
  : driveworksApiWrapper_(driveworksApiWrapper), config_(config), interface_(interface), link_(link), nh_(*nodehandle)
{
  // Read params from yaml
  for (YAML::const_iterator param_it = config_["parameters"].begin(); param_it != config_["parameters"].end();
       ++param_it)
  {
    params_ += param_it->first.as<std::string>() + "=" + param_it->second.as<std::string>() + ",";
  }

  params_ += "interface=csi-" + interface_ + ",";
  params_ += "link=" + link_;
  ROS_DEBUG_STREAM(params_);

  sensorParams_.parameters = params_.c_str();
  sensorParams_.protocol = "camera.gmsl";
  CHECK_DW_ERROR_ROS(dwSAL_createSensor(&sensorHandle_, sensorParams_, driveworksApiWrapper_->sal_handle_));

  // Setup image and camera properties
  CHECK_DW_ERROR_ROS(
      dwSensorCamera_getImageProperties(&imageProperties_, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, sensorHandle_));
  CHECK_DW_ERROR_ROS(dwSensorCamera_getSensorProperties(&cameraProperties_, sensorHandle_));

  // Surface type init
  NVM_SURF_FMT_SET_ATTR_YUV(attrs_, YUV, 422, PLANAR, UINT, 8, PL);
  surfaceType_ = NvMediaSurfaceFormatGetType(attrs_, 7);

  // ROS
  frame_ << "interface" + interface_ + "_link" + link_;
  pub_compressed =
      nh_.advertise<sensor_msgs::CompressedImage>(config_["topic"].as<std::string>() + "/image/compressed", 0);
  pub_info = nh_.advertise<sensor_msgs::CameraInfo>(config_["topic"].as<std::string>() + "/camera_info", 0);

  // Calibration
  nh_.param<std::string>("calib_dir_path", calibDirPath_,
                         ros::package::getPath(ros::this_node::getName().substr(1)) + "/calib/");
  camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
      ros::NodeHandle(config_["topic"].as<std::string>()), frame_.str());
  cam_info_file_ << "file://" << calibDirPath_ << frame_.str() << ".yaml";
  if (camera_info_manager_->validateURL(cam_info_file_.str()) &&
      camera_info_manager_->loadCameraInfo(cam_info_file_.str()))
  {
    camera_info_ = camera_info_manager_->getCameraInfo();
  }

  // Encoder
  nvMediaJPGEncoder_ = std::make_unique<NvMediaJPGEncoder>(&surfaceType_);

  ROS_DEBUG_STREAM("Camera on interface : " << interface_ << ", link : " << link_ << " initialized successfully!");
}

Camera::~Camera()
{
  dwSAL_releaseSensor(sensorHandle_);
  ROS_DEBUG("CAMERA RELEASED !");
}

void Camera::start()
{
  // Start the sensor
  CHECK_DW_ERROR_ROS(dwSensor_start(sensorHandle_));
}

bool Camera::get_last_frame() {
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

  // Get image from cameraFrameHandle
  CHECK_DW_ERROR_ROS_MINOR(dwSensorCamera_getImage(&imageHandleOriginal_, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, cameraFrameHandle_));

  // Return cameraFrameHandle_ back to sensor
  CHECK_DW_ERROR_ROS(dwSensorCamera_returnFrame(&cameraFrameHandle_));

  CHECK_DW_ERROR_ROS(dwImage_getTimestamp(&timestamp_, imageHandleOriginal_));
  imageStamp_ = ros::Time((double)timestamp_ * 10e-7);

  CHECK_DW_ERROR_ROS(dwImage_getNvMedia(&image_nvmedia_, imageHandleOriginal_));

  nvMediaJPGEncoder_->feed_frame(image_nvmedia_);
  while (!nvMediaJPGEncoder_->bits_available()) {}
  nvMediaJPGEncoder_->pull_bits();
}

void Camera::encode() {
  nvMediaJPGEncoder_->feed_frame(image_nvmedia_);
  while (!nvMediaJPGEncoder_->bits_available()) {}
  nvMediaJPGEncoder_->pull_bits();
}

void Camera::publish()
{
  header_.stamp = imageStamp_;
  header_.frame_id = frame_.str();
  img_msg_compressed_.data.assign(nvMediaJPGEncoder_->get_image().get(), nvMediaJPGEncoder_->get_image().get() + nvMediaJPGEncoder_->get_count_bytes());
  img_msg_compressed_.header = header_;
  img_msg_compressed_.format = "jpeg";
  pub_compressed.publish(img_msg_compressed_);

  camera_info_.header = header_;
  pub_info.publish(camera_info_);
}