// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#include "cameras/CameraBase.h"

CameraBase::CameraBase(DriveworksApiWrapper* driveworksApiWrapper, const YAML::Node& config,
                       const std::string interface, const std::string link, ros::NodeHandle* nodehandle)
  : driveworksApiWrapper_(driveworksApiWrapper), config_(config), interface_(interface), link_(link), nh_(*nodehandle)
{
  nh_.param<int>("framerate", framerate_, camera_common::DEFAULT_FRAMERATE);
  nh_.param<int>("output_width", width_, camera_common::DEFAULT_WIDTH);
  nh_.param<int>("output_height", height_, camera_common::DEFAULT_HEIGHT);

  width_ = 800;
  height_ = 600;

  // Read params from yaml.
  for (YAML::const_iterator param_it = config_["parameters"].begin(); param_it != config_["parameters"].end();
       ++param_it) {
    params_ += param_it->first.as<std::string>() + "=" + param_it->second.as<std::string>() + ",";
  }

  params_ += "interface=csi-" + interface_ + ",";
  params_ += "link=" + link_;
  sensorParams_.parameters = params_.c_str();
  sensorParams_.protocol = "camera.gmsl";
  CHK_DW(dwSAL_createSensor(&sensorHandle_, sensorParams_, driveworksApiWrapper_->sal_handle_));

  // ROS.
  frame_ = "interface" + interface_ + "_link" + link_;
  pub_info_ = nh_.advertise<sensor_msgs::CameraInfo>(config_["topic"].as<std::string>() + "/camera_info", 1);

  // Calibration.
  nh_.param<std::string>("calib_dir_path", calibDirPath_,
                         ros::package::getPath(ros::this_node::getName().substr(1)) + "/calib/");
  camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
      ros::NodeHandle(config_["topic"].as<std::string>()), frame_);
  cam_info_file_ << "file://" << calibDirPath_ << frame_ << ".yaml";
  if (camera_info_manager_->validateURL(cam_info_file_.str()) &&
      camera_info_manager_->loadCameraInfo(cam_info_file_.str())) {
    camera_info_ = camera_info_manager_->getCameraInfo();
  }

  // Setup images and camera properties.
  CHK_DW(dwSensorCamera_getSensorProperties(&cameraProperties_, sensorHandle_));
  CHK_DW(dwSensorCamera_getImageProperties(&cameraImgProps_, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, sensorHandle_));

  // Image transformer not needed if output image dimensions the same as camera.
  if (true || width_ != cameraImgProps_.width || height_ != cameraImgProps_.width) {
    imageTransformer_ = std::make_unique<ImageTransformer>(driveworksApiWrapper_, width_, height_);
    CHK_DW(dwImage_create(&imgTransformed_, camera_common::get_rgba_pitch_img_prop(width_, height_),
                          driveworksApiWrapper_->context_handle_));
  }

  // Create camera image
  CHK_DW(dwImage_create(&imgOutOfCamera_, cameraImgProps_, driveworksApiWrapper_->context_handle_));

  ROS_DEBUG_STREAM("CameraH264 on interface : " << interface_ << ", link : " << link_ << " initialized successfully!");
}

CameraBase::~CameraBase()
{
  dwSAL_releaseSensor(sensorHandle_);
  dwImage_destroy(imgOutOfCamera_);
  dwImage_destroy(imgTransformed_);
}

void CameraBase::start()
{
  CHK_DW(dwSensor_start(sensorHandle_));
}

bool CameraBase::poll()
{
  dwStatus status_old = DW_NOT_READY;

  while (true) {
    status_ = dwSensorCamera_readFrameNew(&cameraFrameHandle_, 0, sensorHandle_);

    if (status_ == DW_NOT_READY) {
      throw NvidiaGmslDriverRosMinorException("Unable to get the frame from the camera.");
    }

    if (status_ == DW_TIME_OUT && status_old == DW_SUCCESS) {
      return true;
    }

    status_old = status_;
  }
}

void CameraBase::run_pipeline()
{
  poll();
  preprocess();
  encode();
  CHK_DW(dwSensorCamera_returnFrame(&cameraFrameHandle_));
  publish();
}

void CameraBase::preprocess()
{
  CHK_DW(dwSensorCamera_getImage(&imgOutOfCamera_, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, cameraFrameHandle_));
  CHK_DW(dwImage_getTimestamp(&timestamp_, imgOutOfCamera_));

  // todo clean condition
  if (true || width_ != cameraImgProps_.width || height_ != cameraImgProps_.height) {
    // requires transformation
    imageTransformer_->transform_image(&imgOutOfCamera_, &imgTransformed_);
    return;
  }
}
