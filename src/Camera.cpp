// Created by Maxandre Ogeret.
// (c) 2021 University of Tartu - Autonomous Driving Lab.

#include "Camera.h"

Camera::Camera(std::shared_ptr<DriveworksApiWrapper> driveworksApiWrapper, const YAML::Node config, const std::string interface, const std::string link, ros::NodeHandle *nodehandle)
    : driveworksApiWrapper_(driveworksApiWrapper), config_(config), interface_(interface), link_(link), nh_(*nodehandle) {

  // Read params from yaml
  for (YAML::const_iterator param_it = config_["parameters"].begin(); param_it != config_["parameters"].end(); ++param_it) {
    params_ += param_it->first.as<std::string>() + "=" + param_it->second.as<std::string>() + ",";
  }

  params_ += "interface=csi-" + interface_ + ",";
  params_ += "link=" + link_;
  ROS_DEBUG_STREAM(params_);

  sensorParams_.parameters = params_.c_str();
  sensorParams_.protocol = "camera.gmsl";
  CHECK_DW_ERROR_ROS(dwSAL_createSensor(&sensorHandle_, sensorParams_, driveworksApiWrapper_->sal_handle_));

  // Setup image and camera properties
  CHECK_DW_ERROR_ROS(dwSensorCamera_getImageProperties(&imageProperties_, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, sensorHandle_));
  CHECK_DW_ERROR_ROS(dwSensorCamera_getSensorProperties(&cameraProperties_, sensorHandle_));

  // JPG Init
  jpegImage_ = std::make_unique<uint8_t[]>(maxJpegBytes_);
  NVM_SURF_FMT_SET_ATTR_YUV(attrs_, YUV, 422, PLANAR, UINT, 8, PL);
  surfaceType_ = NvMediaSurfaceFormatGetType(attrs_, 7);

  // Nvmedia init
  nvmediaDevice_ = NvMediaDeviceCreate();
  nvMediaIjpe_ = NvMediaIJPECreate(nvmediaDevice_,
                                   surfaceType_,
                                   (uint8_t) 1,
                                   maxJpegBytes_);

  // ROS
  frame_ << "interface" + interface_ + "_link" + link_;
  pub_compressed = nh_.advertise<sensor_msgs::CompressedImage>(config_["topic"].as<std::string>() + "/image_raw/compressed", 1024);
  pub_info = nh_.advertise<sensor_msgs::CameraInfo>(config_["topic"].as<std::string>() + "/camera_info", 1024);

  //Calibration
  camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(ros::NodeHandle(config_["topic"].as<std::string>()), frame_.str());
  cam_info_file_ << "file://" << ros::package::getPath(ros::this_node::getName().substr(1)) << "/calib/" << frame_.str() << ".yaml";
  if (camera_info_manager_->validateURL(cam_info_file_.str()) &&
      camera_info_manager_->loadCameraInfo(cam_info_file_.str())) {
    camera_info_ = camera_info_manager_->getCameraInfo();
  }

  ROS_INFO_STREAM("Camera on interface : " << interface_ << ", link : " << link_ << " initialized successfully!");
}

Camera::~Camera() {
  dwSAL_releaseSensor(sensorHandle_);
  ROS_DEBUG("CAMERA RELEASED !");
}

bool Camera::start() {
  // Start the sensor
  dwSensor_start(sensorHandle_);
}

bool Camera::poll() {
  status_ = dwSensorCamera_readFrameNew(&cameraFrameHandle_, 0, sensorHandle_);
  if (status_ == DW_NOT_READY) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    return false;
  }

  if (status_ == DW_TIME_OUT) {
    ROS_WARN_STREAM("DROPPED FRAME FOR : " << frame_.str());
    return false;
  }

  if (status_ != DW_SUCCESS) {
    ROS_FATAL_STREAM("FAILED TO READ FRAME FOR : " << frame_.str());
    CHECK_DW_ERROR_ROS(status_);
    return false;
  }

  // Get image from cameraFrameHandle
  status_ = dwSensorCamera_getImage(&imageHandleOriginal_, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, cameraFrameHandle_);
  if (status_ != DW_SUCCESS) {
    ROS_FATAL_STREAM("dwSensorCamera_getImage() Failed");
    return false;
  }

  status_ = dwImage_create(&imageHandle_, imageProperties_, driveworksApiWrapper_->context_handle_);
  if (status_ != DW_SUCCESS) {
    ROS_FATAL_STREAM("dwImage_create() Failed");
    return false;
  }

  status_ = dwImage_copyConvert(imageHandle_, imageHandleOriginal_, driveworksApiWrapper_->context_handle_);
  if (status_ != DW_SUCCESS) {
    ROS_FATAL_STREAM("dwImage_copyConvert() Failed");
    return false;
  }

  // Return cameraFrameHandle_ back to sensor
  status_ = dwSensorCamera_returnFrame(&cameraFrameHandle_);
  if (status_ != DW_SUCCESS) {
    std::cout << "dwSensorCamera_returnFrame() Failed" << std::endl;
    return false;
  }

  status_ = dwImage_getTimestamp(&timestamp_, imageHandleOriginal_);
  if (status_ != DW_SUCCESS) {
    ROS_FATAL_STREAM("dwImage_getTimestamp() Failed");
    return false;
  }
  imageStamp_ = ros::Time((double) timestamp_ * 10e-7);

  dwImageNvMedia *image_nvmedia;
  status_ = dwImage_getNvMedia(&image_nvmedia, imageHandle_);
  if (status_ != DW_SUCCESS) {
    ROS_FATAL_STREAM("dwImage_getNvMedia() Failed");
    return false;
  }

  nvMediaStatus_ = NvMediaIJPEFeedFrame(nvMediaIjpe_, image_nvmedia->img, 70);
  if (nvMediaStatus_ != NVMEDIA_STATUS_OK) {
    ROS_FATAL_STREAM("NvMediaIJPEFeedFrame() failed: " << std::to_string(nvMediaStatus_));
    return false;
  }

  do {
    nvMediaStatus_ = NvMediaIJPEBitsAvailable(nvMediaIjpe_, &countByteJpeg_, NVMEDIA_ENCODE_BLOCKING_TYPE_NEVER, 0);
  } while (nvMediaStatus_ != NVMEDIA_STATUS_OK);

  nvMediaStatus_ = NvMediaIJPEGetBits(nvMediaIjpe_, &countByteJpeg_, jpegImage_.get(), 0);
  if (nvMediaStatus_ != NVMEDIA_STATUS_OK) {
    ROS_FATAL_STREAM("NvMediaIJPEGetBits() failed: " << std::to_string(nvMediaStatus_));
    return false;
  }

  return true;
}

void Camera::publish() {
  sensor_msgs::CompressedImage img_msg_compressed;
  img_msg_compressed.data.resize(countByteJpeg_);
  std::copy(jpegImage_.get(), jpegImage_.get()+countByteJpeg_, img_msg_compressed.data.begin());
  std_msgs::Header header;
  header.stamp = imageStamp_;
  header.frame_id = frame_.str();
  img_msg_compressed.header = header;
  img_msg_compressed.format = "jpeg";
  pub_compressed.publish(img_msg_compressed);

  pub_info.publish(camera_info_);
}

void Camera::clean() {
  status_ = dwImage_destroy(imageHandle_);
  if (status_ != DW_SUCCESS) {
    ROS_ERROR_STREAM("dwImage_destroy imageHandle_ Failed");
  }
}