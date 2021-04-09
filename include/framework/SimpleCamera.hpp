/////////////////////////////////////////////////////////////////////////////////////////
// This code contains NVIDIA Confidential Information and is disclosed
// under the Mutual Non-Disclosure Agreement.
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. No third party distribution is allowed unless
// expressly authorized by NVIDIA.  Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2015-2017 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#ifndef COMMON_SIMPLECAMERA_HPP_
#define COMMON_SIMPLECAMERA_HPP_

// Driveworks
#include <dw/core/Context.h>
#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>
#include <dw/image/Image.h>
#include <dw/isp/SoftISP.h>

// C++ Std
#include <memory>
#include <vector>
#include <type_traits>
#include <chrono>
#include <thread>

// Common
#include <framework/Checks.hpp>
#include <framework/SimpleStreamer.hpp>

namespace dw_samples
{
namespace common
{

//-------------------------------------------------------------------------------
/**
* Class to process camera frame. Supports streaming and converting
* (once, in that order) so the returned image is in the expected format.
* The real type matches the type requested in the output properties.
*/
class CameraFramePipeline
{
public:

    CameraFramePipeline(const dwImageProperties &inputImageProperties,
                   dwCameraOutputType outputType,
                   dwContextHandle_t ctx);

    virtual ~CameraFramePipeline();

    /// Sets up output properties. Initializes streamers and converters
    /// if required.
    void setOutputProperties(const dwImageProperties& outputImageProperties);

    /// Performs single camera frame processing.
    virtual void processFrame(dwCameraFrameHandle_t cameraFrame);

    /// Acquire the latest processed frame. Only valid when GL
    /// output is enabled.
    dwImageHandle_t getFrameRgba();
    dwImageHandle_t getFrameGL();
    dwImageHandle_t getFrame();


    bool isGLOutputEnabled() const { return m_streamerGL.get() != nullptr; }

    /// Enables conversion and streaming frame directly to GL
    void enableGLOutput();

    const dwImageProperties& getImageProperties() const { return m_inputImgProps; }
    virtual const dwImageProperties &getOutputProperties() const { return m_outputImgProps; }

    /// virtual method indicating softISP is present, always false for SimpleCamera
    virtual bool isSoftISPEnabled() const { return false; }

protected:

    dwContextHandle_t m_ctx;

    dwCameraOutputType m_outputType;
    dwImageProperties m_inputImgProps;
    dwImageProperties m_outputImgProps;

    std::unique_ptr<SimpleImageStreamer<>> m_streamer;
    dwImageHandle_t m_converter;

    std::unique_ptr<SimpleImageStreamer<>> m_streamerGL;
    dwImageHandle_t m_image;
    dwImageHandle_t m_imageRgba;
    dwImageHandle_t m_imageRgbaGL;
};


/**
 * Extension of the CameraFramePipeline that applies the raw pipeline to the camera frame. Result of a readFrame is
 * a RCB/RCC frame.
*/
class RawCameraFramePipeline: public CameraFramePipeline
{
public:

    RawCameraFramePipeline(const dwImageProperties& inputImageProperties,
                      dwCameraType cameraType,
                      dwCameraOutputType outputType,
                      cudaStream_t cudaStream,
                      const dwSoftISPParams& ispParams,
                      dwSoftISPDemosaicMethod demosaicMethod,
                      bool localToneMapping,
                      dwContextHandle_t ctx);

    ~RawCameraFramePipeline() override;

    const dwImageProperties &getOutputProperties() const override { return m_rawOutputProperties; }

    void setOutputISPFormat(dwImageFormat outputISPFormat);
    void setRawOutputProperties(const dwImageProperties& outputProperties);

    void processFrame(dwCameraFrameHandle_t cameraFrame) override final;

    bool isSoftISPEnabled() const override final { return (m_softISP != nullptr); }

private:

    dwSoftISPHandle_t m_softISP;
    dwImageHandle_t m_rawImage;
    dwImageHandle_t m_RGBImage;

    dwImageProperties m_rawOutputProperties;
    dwImageHandle_t m_finalImage;

    bool m_doTonemap;
};

/**
* Simple class to get images from a camera. It supports streaming and converting (once, in that order)
* so the returned image is in the expected format. It returns the generic dwImageGeneric type that points
* to an underlying concrete dwImageXXX struct. The real type matches the type requested in the output properties.
* The class uses CameraFramePipeline for the processing.
*
* Usage:
* \code
* SimpleCamera camera(propsOut, sensorParams, sal, ctx);
*
* for(dwImageGeneric *img = camera.readFrame(); img!=nullptr; img=camera.readFrame())
* {
*   // Do things with img
* }
*
* \endcode
*
* NOTE: for tutorials and best practices about Camera sensors, please see sensors/camera samples
*/
class SimpleCamera
{
  public:
    /// creates a simple camera that outputs a frame with the properties of the camera image
    SimpleCamera(const dwSensorParams &params, dwSALHandle_t sal, dwContextHandle_t ctx,
                 dwCameraOutputType outputType = DW_CAMERA_OUTPUT_NATIVE_PROCESSED);
    /**
     * creates a simple camera and also sets up image streamer and format converter to output a
     * converted image, with properties different from the properties of the camera image
    **/
    SimpleCamera(const dwImageProperties &outputProperties, const dwSensorParams &params, dwSALHandle_t sal,
                 dwContextHandle_t ctx, dwCameraOutputType outputType = DW_CAMERA_OUTPUT_NATIVE_PROCESSED);

    virtual ~SimpleCamera();

    /**
     * sets up streamer and converter to be used when acquiring a frame if outputProperties are different
     * from input properties
    **/
    void setOutputProperties(const dwImageProperties &outputProperties);

    const dwCameraProperties &getCameraProperties() const { return m_cameraProperties; }
    const dwImageProperties &getImageProperties() const { return m_framePipeline->getImageProperties(); }
    virtual const dwImageProperties &getOutputProperties() const {return m_framePipeline->getOutputProperties();}

    virtual dwImageHandle_t readFrame();

    /// Seeking functionality
    bool enableSeeking(size_t& frameCount, dwTime_t& startTimestamp, dwTime_t& endTimestamp);
    void seekToTime(dwTime_t timestamp);
    void seekToFrame(size_t frameIdx);


    /// Releases the frame returned by readFrame. Calling this is optional.
    void releaseFrame();

    void resetCamera();

    /// Enables conversion and streaming directly to GL for each frame read
    /// After this is enabled, getFrameGL() will return the GL image for the last read frame.
    void enableGLOutput();

    bool isGLOutputEnabled() const { return m_framePipeline->isGLOutputEnabled(); }

    /// Returns the frame converted to RGBA format of the same type as the input image
    /// Only valid when GL output has been enabled
    dwImageHandle_t getFrameRgba() const { return m_framePipeline->getFrameRgba(); }

    /// Returns the frame converted to RGBA format as a GL frame
    /// Only valid when GL output has been enabled
    dwImageHandle_t getFrameRgbaGL() const { return m_framePipeline->getFrameGL(); }

protected:

    /// Constructor is only called from derived RawSimpleCamera
    SimpleCamera(const dwSensorParams &params, dwSALHandle_t sal, dwContextHandle_t ctx, cudaStream_t stream,
                 dwCameraOutputType outputType, dwSoftISPDemosaicMethod demosaicMethod, bool localToneMapping);

    void createSensor(dwImageProperties& imageProps, const dwSensorParams &params, dwCameraOutputType outputType);

    dwSALHandle_t m_sal;

    dwSensorHandle_t m_sensor;
    dwCameraProperties m_cameraProperties;

    dwCameraFrameHandle_t m_pendingFrame;

    std::unique_ptr<CameraFramePipeline> m_framePipeline;

    bool m_started;
};

/**
 * Extension of the SimpleCamera that reads a RAW image and applies the raw pipeline. Calling readFrame will
 * a RCB/RCC frame.
 *
 * NOTE for tutorial and details about raw pipeline and raw cameras, see sample_camera_gmsl_raw and sample_raw_pipeline
 */
class RawSimpleCamera : public SimpleCamera
{
public:
    RawSimpleCamera(const dwSensorParams &params, dwSALHandle_t sal, dwContextHandle_t ctx, cudaStream_t stream, 
               dwCameraOutputType outputType, dwSoftISPDemosaicMethod demosaicMethod = DW_SOFTISP_DEMOSAIC_METHOD_DOWNSAMPLE, bool localToneMapping = false);
    
    RawSimpleCamera(const dwImageFormat &outputISPFormat, const dwSensorParams &params, dwSALHandle_t sal,
                    dwContextHandle_t ctx, cudaStream_t stream, dwCameraOutputType outputType,
                    dwSoftISPDemosaicMethod demosaicMethod = DW_SOFTISP_DEMOSAIC_METHOD_DOWNSAMPLE);

    /**
     * Creates a raw simple camera and also sets up format converter to output a
     * converted image, with properties different from the properties of the camera image
     * Note, the output resolution depends on the demosaicMethod.
    **/
    RawSimpleCamera(const dwImageProperties &outputProperties, const dwSensorParams &params, dwSALHandle_t sal, dwContextHandle_t ctx, cudaStream_t stream, 
                    dwCameraOutputType outputType, dwSoftISPDemosaicMethod demosaicMethod);
};
}
}

#endif
