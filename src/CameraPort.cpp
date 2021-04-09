#include "CameraPort.h"
#include <dw/core/Status.h>
#include <iostream>
#include <StopWatch.h>
#include <thread>
#include <utility>

namespace DriveWorks {
CameraPort::CameraPort(
    dwSensorHandle_t sensor_handle,
    bool debug_mode,
    int port,
    const std::string &caminfo_folder,
    PrintEventHandler::Ptr printer)
    : debug_mode(debug_mode),
      sensor_handle_(sensor_handle),
      port(port),
      printer_(printer) {
  std::cout << "------------------ CameraPort::CameraPort() --------------------" << std::endl;
  name_pretty_ = "Port #" + std::to_string(port);

  auto report_status = [&](const std::string &subject, const dwStatus &status) {
    if (status != DW_SUCCESS) {
      std::string msg_error = subject + " status: " + dwGetStatusName(status);
      printer_->Print(name_pretty_, msg_error);
      return false;
    }
  };

  dwStatus status;
  status = dwSensorCamera_getImageProperties(&image_properties_,
                                             DW_CAMERA_OUTPUT_NATIVE_PROCESSED,
                                             sensor_handle);
  report_status("dwSensorCamera_getImageProperties", status);

  status = dwSensorCamera_getSensorProperties(&camera_properties_, sensor_handle);
  report_status("dwSensorCamera_getSensorProperties", status);

  std::cout << "GetSiblingCount(): " << GetSiblingCount() << std::endl;
  Cameras.resize(GetSiblingCount());

  for (uint32_t ind_camera = 0; ind_camera < GetSiblingCount(); ind_camera++) {
    const std::string topic = "port_" + std::to_string(port) + "/camera_" + std::to_string(ind_camera);
    const std::string camera_frame_id = "port_" + std::to_string(port) + "/camera_" + std::to_string(ind_camera);
    const std::string cam_info_file = "file://" + caminfo_folder + std::to_string(port) + std::to_string(ind_camera) + "_calibration.yml";
    Camera &camera = Cameras[ind_camera];
    camera.Index = ind_camera;
    camera.NamePretty = name_pretty_ + " " + "Cam #" + std::to_string(camera.Index);
    camera.OpenCvConnector = std::make_shared<OpenCVConnector>(topic, camera_frame_id, cam_info_file, 1024);
  }
}

dwStatus CameraPort::Start(const dwContextHandle_t &context_handle) {
  for (size_t ind_camera = 0; ind_camera < GetSiblingCount(); ++ind_camera) {
    Camera &camera = Cameras[ind_camera];
    const uint32_t max_jpeg_bytes = 3 * 1290 * 1208;
    camera.JpegImage = (uint8_t *) malloc(max_jpeg_bytes);

    camera.NvmediaDevice = nullptr;
    camera.NvmediaDevice = NvMediaDeviceCreate();
    if (!camera.NvmediaDevice) {
      printer_->Print(camera.NamePretty, "NvMediaDeviceCreate failed.");
      exit(EXIT_FAILURE);
    }

    NvMediaSurfFormatAttr attrs[7];
    NVM_SURF_FMT_SET_ATTR_YUV(attrs, YUV, 422, PLANAR, UINT, 8, PL);
    NvMediaSurfaceType surface_type = NvMediaSurfaceFormatGetType(attrs, 7);
    camera.NvMediaIjpe = nullptr;
    camera.NvMediaIjpe = NvMediaIJPECreate(camera.NvmediaDevice,
                                           surface_type,
                                           (uint8_t) 1,
                                           max_jpeg_bytes);
    if (!camera.NvMediaIjpe) {
      printer_->Print(camera.NamePretty, "NvMediaIJPECreate failed.");
      exit(EXIT_FAILURE);
    }

  }
  return dwSensor_start(GetSensorHandle());
}

int CameraPort::GetSiblingCount() {
  return camera_properties_.siblings;
}

dwSensorHandle_t CameraPort::GetSensorHandle() const {
  return sensor_handle_;
}

void CameraPort::ProcessCameraStreams(std::atomic_bool &is_running, const dwContextHandle_t &context_handle) {
  for (int ind_camera = 0; ind_camera < Cameras.size(); ind_camera++) {
    Camera &camera = Cameras[ind_camera];

    dwStatus status;

    dwCameraFrameHandle_t camera_frame_handle;

    dwImageHandle_t image_handle;
    dwImageHandle_t image_handle_original;

    status = dwSensorCamera_readFrame(&camera_frame_handle, ind_camera, 0, sensor_handle_);
    if (status != DW_SUCCESS) {
      continue;
    }

    status = dwSensorCamera_getImage(&image_handle_original, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, camera_frame_handle);
    if (status != DW_SUCCESS) {
      std::cout << "dwSensorCamera_getImage() Failed" << std::endl;
      is_running = false;
    }

    status = dwImage_create(&image_handle, image_properties_, context_handle);
    if (status != DW_SUCCESS) {
      std::cout << "dwImage_create() Failed" << std::endl;
      is_running = false;
    }

    status = dwImage_copyConvert(image_handle, image_handle_original, context_handle);
    if (status != DW_SUCCESS) {
      std::cout << "dwImage_copyConvert() Failed" << std::endl;
      is_running = false;
    }

    status = dwSensorCamera_returnFrame(&camera_frame_handle);
    if (status != DW_SUCCESS) {
      std::cout << "dwSensorCamera_returnFrame() Failed" << std::endl;
      is_running = false;
    }

    dwTime_t timestamp;
    status = dwImage_getTimestamp(&timestamp, image_handle_original);
    if (status != DW_SUCCESS) {
      std::cout << "dwImage_getTimestamp() Failed" << std::endl;
      is_running = false;
    }

    Camera::ImageWithStamp image_with_stamp;
    image_with_stamp.image_handle = image_handle;
    image_with_stamp.time_stamp = ros::Time((double) timestamp * 10e-7);

    dwImageNvMedia *image_nvmedia;
    status = dwImage_getNvMedia(&image_nvmedia, image_with_stamp.image_handle);
    if (status != DW_SUCCESS) {
      std::cout << "dwImage_getNvMedia() Failed" << std::endl;
      is_running = false;
    }

    NvMediaStatus nvStatus = NvMediaIJPEFeedFrame(camera.NvMediaIjpe, image_nvmedia->img, 70);
    if (nvStatus != NVMEDIA_STATUS_OK) {
      std::cout << "NvMediaIJPEFeedFrame() failed: " << std::to_string(nvStatus) << std::endl;
      is_running = false;
    }

    do {
      nvStatus = NvMediaIJPEBitsAvailable(camera.NvMediaIjpe, &camera.CountByteJpeg, NVMEDIA_ENCODE_BLOCKING_TYPE_NEVER, 0);
    } while (nvStatus != NVMEDIA_STATUS_OK);

    nvStatus = NvMediaIJPEGetBits(camera.NvMediaIjpe, &camera.CountByteJpeg, camera.JpegImage, 0);
    if (nvStatus != NVMEDIA_STATUS_OK) {
      std::cout << "NvMediaIJPEGetBits() failed: " << std::to_string(nvStatus) << std::endl;
      is_running = false;
    }

    camera.OpenCvConnector->WriteToJpeg(camera.JpegImage,
                                        camera.CountByteJpeg,
                                        image_with_stamp.time_stamp);

    status = dwImage_destroy(image_with_stamp.image_handle);
    if (status != DW_SUCCESS) {
      std::cout << "dwImage_destroy() Failed" << std::endl;
      is_running = false;
    }
  }
}


void CameraPort::CleanUp() {
  std::cout << "------------------ CameraPort::CleanUp() --------------------" << std::endl;

  printer_->Print(name_pretty_, "clean up has started!");
  dwStatus status;
  status = dwSensor_stop(GetSensorHandle());
  if (status != DW_SUCCESS) {
    printer_->Print(name_pretty_, "dwSensor_stop: " + std::string(dwGetStatusName(status)));
  }
  printer_->Print(name_pretty_, "dwSensor_stop");
  status = dwSAL_releaseSensor(GetSensorHandle());
  if (status != DW_SUCCESS) {
    printer_->Print(name_pretty_, "dwSAL_releaseSensor: " + std::string(dwGetStatusName(status)));
  }
  printer_->Print(name_pretty_, "dwSAL_releaseSensor");
  for (auto &camera : Cameras) {
    NvMediaIJPEDestroy(camera.NvMediaIjpe);
    printer_->Print(camera.NamePretty, "NvMediaIJPEDestroy");
    NvMediaDeviceDestroy(camera.NvmediaDevice);
    printer_->Print(camera.NamePretty, "NvMediaDeviceDestroy");
  }

}

CameraPort::~CameraPort() {
  std::cout << "------------------ CameraPort::~CameraPort() --------------------" << std::endl;
  printer_->Print(name_pretty_, "Destructor is called!");
  CleanUp();
}

}