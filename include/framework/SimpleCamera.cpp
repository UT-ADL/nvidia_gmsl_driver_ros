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

#include "SimpleCamera.hpp"

#include <framework/Log.hpp>

namespace dw_samples
{
namespace common
{

///////////////////////////////////////////////////////////////////////////////
/// CameraFramePipeline
///////////////////////////////////////////////////////////////////////////////

CameraFramePipeline::CameraFramePipeline(const dwImageProperties &inputImageProperties,
                               dwCameraOutputType outputType,
                               dwContextHandle_t ctx)
    : m_ctx(ctx)
    , m_outputType(outputType)
    , m_inputImgProps(inputImageProperties)
    , m_outputImgProps(inputImageProperties)
    , m_converter(DW_NULL_HANDLE)
    , m_image(DW_NULL_HANDLE)
    , m_imageRgba(DW_NULL_HANDLE)
    , m_imageRgbaGL(DW_NULL_HANDLE)
{}

CameraFramePipeline::~CameraFramePipeline()
{
    if (m_converter != DW_NULL_HANDLE)
        dwImage_destroy(m_converter);

    if (m_imageRgba != DW_NULL_HANDLE)
        dwImage_destroy(m_imageRgba);
}

void CameraFramePipeline::setOutputProperties(const dwImageProperties &outputImageProperties)
{
    m_outputImgProps = outputImageProperties;
    m_outputImgProps.width = m_inputImgProps.width;
    m_outputImgProps.height = m_inputImgProps.height;

    if(m_inputImgProps.type != m_outputImgProps.type)
    {
        m_streamer.reset(new SimpleImageStreamer<>(m_inputImgProps, m_outputImgProps.type, 60000, m_ctx));
    }

    if (m_inputImgProps.format != m_outputImgProps.format)
    {
        dwImage_create(&m_converter, m_outputImgProps, m_ctx);
    }

}

void CameraFramePipeline::enableGLOutput()
{
    const dwImageProperties& outProps = getOutputProperties();

     dwImageProperties propsRgba{};
     propsRgba.format = DW_IMAGE_FORMAT_RGBA_UINT8;
     propsRgba.type = outProps.type;
     propsRgba.width = outProps.width;
     propsRgba.height = outProps.height;
     propsRgba.memoryLayout = outProps.memoryLayout;

     dwImage_create(&m_imageRgba, propsRgba, m_ctx);
     m_streamerGL.reset(new SimpleImageStreamer<>(propsRgba, DW_IMAGE_GL, 60000, m_ctx));
}

dwImageHandle_t CameraFramePipeline::getFrame()
{
    return m_image;
}

dwImageHandle_t CameraFramePipeline::getFrameGL()
{
    if(!isGLOutputEnabled()) {
        logWarn("CameraFramePipeline: GL output is not enabled. Did you forget to call enableGLOutput()?\n");
    }
    return m_imageRgbaGL;
}

dwImageHandle_t CameraFramePipeline::getFrameRgba()
{
    if(!isGLOutputEnabled()) {
        logWarn("CameraFramePipeline: GL output is not enabled. Did you forget to call enableGLOutput()?\n");
    }
    return m_imageRgba;
}

void CameraFramePipeline::processFrame(dwCameraFrameHandle_t cameraFrame)
{
    dwImageHandle_t img;
    CHECK_DW_ERROR(dwSensorCamera_getImage(&img, m_outputType, cameraFrame));

    m_image = img;
    if(m_streamer)
    {
        m_image = m_streamer->post(img);
    }

    if (m_converter)
    {
        dwImage_copyConvert(m_converter, m_image, m_ctx);
        m_image = m_converter;
    }

    // OpenGL
    if(isGLOutputEnabled() && !isSoftISPEnabled())
    {
        dwImage_copyConvert(m_imageRgba, m_image, m_ctx);
        m_imageRgbaGL = m_streamerGL->post(m_imageRgba);
    }
}


///////////////////////////////////////////////////////////////////////////////
/// RawCameraFramePipeline
///////////////////////////////////////////////////////////////////////////////

RawCameraFramePipeline::RawCameraFramePipeline(const dwImageProperties &inputImageProperties,
                                     dwCameraType cameraType,
                                     dwCameraOutputType outputType,
                                     cudaStream_t cudaStream,
                                     const dwSoftISPParams &ispParams,
                                     dwSoftISPDemosaicMethod demosaicMethod,
                                     bool localToneMapping,
                                     dwContextHandle_t ctx)
    : CameraFramePipeline(inputImageProperties, DW_CAMERA_OUTPUT_NATIVE_RAW, ctx)
    , m_softISP(DW_NULL_HANDLE)
    , m_rawImage(DW_NULL_HANDLE)
    , m_RGBImage(DW_NULL_HANDLE)
    , m_finalImage(DW_NULL_HANDLE)
    , m_doTonemap(false)
{
    if (outputType == DW_CAMERA_OUTPUT_NATIVE_PROCESSED) {
        m_doTonemap = true;
    }

    // set output type so it streams to cuda after reading raw frame
    dwImageProperties cameraImageProps = CameraFramePipeline::m_inputImgProps;
    cameraImageProps.type = DW_IMAGE_CUDA;
    CameraFramePipeline::setOutputProperties(cameraImageProps);

    dwSoftISPParams softISPParams = ispParams;
    if (localToneMapping)
        softISPParams.method = DW_TONEMAP_METHOD_LTM;
    CHECK_DW_ERROR(dwSoftISP_initialize(&m_softISP, &softISPParams, ctx));

    // Initialize Raw pipeline
    CHECK_DW_ERROR(dwSoftISP_setCUDAStream(cudaStream, m_softISP));

    // AR0144 camera does not support demosaic process type
    if(cameraType != DW_CAMERA_GMSL_AR0144)
    {
        CHECK_DW_ERROR(dwSoftISP_setDemosaicMethod(demosaicMethod, m_softISP));
        CHECK_DW_ERROR(dwSoftISP_getDemosaicImageProperties(&m_rawOutputProperties, m_softISP));

        // RCB image to get output from the RawPipeline
        CHECK_DW_ERROR(dwImage_create(&m_rawImage, m_rawOutputProperties, m_ctx));
        dwImageCUDA* rawImageCuda;
        dwImage_getCUDA(&rawImageCuda, m_rawImage);

        CHECK_DW_ERROR(dwSoftISP_bindOutputDemosaic(rawImageCuda, m_softISP));

        int32_t processType = DW_SOFTISP_PROCESS_TYPE_DEMOSAIC;
        if(m_doTonemap)
        {
            processType |= DW_SOFTISP_PROCESS_TYPE_TONEMAP;
        }

        dwSoftISP_setProcessType(processType, m_softISP);
    }
    else
    {
        m_rawOutputProperties = cameraImageProps;
        dwSoftISP_setProcessType(DW_SOFTISP_PROCESS_TYPE_TONEMAP, m_softISP);
    }

    if (m_doTonemap) {
        dwImageProperties rgbProps = m_rawOutputProperties;
        rgbProps.format = DW_IMAGE_FORMAT_RGBA_UINT8;

        CHECK_DW_ERROR(dwImage_create(&m_RGBImage, rgbProps, m_ctx));
        dwImageCUDA* RGB_cuda;
        dwImage_getCUDA(&RGB_cuda, m_RGBImage);
        dwSoftISP_bindOutputTonemap(RGB_cuda, m_softISP);
    }
}

RawCameraFramePipeline::~RawCameraFramePipeline()
{
    if (m_finalImage) {
        dwImage_destroy(m_finalImage);
    }

    if (m_rawImage) {
        dwImage_destroy(m_rawImage);
    }

    if (m_RGBImage) {
        dwImage_destroy(m_RGBImage);
    }

    dwSoftISP_release(m_softISP);
}


void RawCameraFramePipeline::setOutputISPFormat(dwImageFormat outputISPFormat)
{
    if (m_rawOutputProperties.format != outputISPFormat)
    {
        if (m_RGBImage)
            CHECK_DW_ERROR(dwImage_destroy(m_RGBImage));

        if (m_doTonemap) {
            dwImageProperties rgbProps = m_rawOutputProperties;
            rgbProps.format = outputISPFormat;

            CHECK_DW_ERROR(dwImage_create(&m_RGBImage, rgbProps, m_ctx));
            dwImageCUDA* RGB_cuda;
            dwImage_getCUDA(&RGB_cuda, m_RGBImage);
            dwSoftISP_bindOutputTonemap(RGB_cuda, m_softISP);
        }
    }
}


void RawCameraFramePipeline::setRawOutputProperties(const dwImageProperties &outputProperties)
{
    if (m_outputImgProps.format != outputProperties.format)
    {
        dwImageProperties newProperties = m_rawOutputProperties;
        newProperties.format = outputProperties.format;

        CHECK_DW_ERROR(dwImage_create(&m_finalImage, newProperties, m_ctx));
        m_rawOutputProperties = newProperties;
    }
}


void RawCameraFramePipeline::processFrame(dwCameraFrameHandle_t cameraFrame)
{
    // see sample raw_pipeline for full use and explanation
    CameraFramePipeline::processFrame(cameraFrame);

    dwImageHandle_t rawImage = getFrame();

    if (rawImage == nullptr) {
        return;
    }

    dwImageHandle_t output = m_rawImage;
    if (m_doTonemap) {
        output = m_RGBImage;
    }

    dwImageCUDA* rawImageCUDA;
    dwImage_getCUDA(&rawImageCUDA, rawImage);
    dwSoftISP_bindInputRaw(rawImageCUDA, m_softISP);

    CHECK_DW_ERROR(dwSoftISP_processDeviceAsync(m_softISP));

    m_image = output;
    if (m_finalImage)  {
        dwImage_copyConvert(m_finalImage, m_image, m_ctx);
        m_image = m_finalImage;
    }

    if(isGLOutputEnabled() && isSoftISPEnabled())
    {
        dwImage_copyConvert(m_imageRgba, m_image, m_ctx);
        m_imageRgbaGL = m_streamerGL->post(m_imageRgba);
    }
}

///////////////////////////////////////////////////////////////////////////////

SimpleCamera::SimpleCamera(const dwSensorParams &params,
                           dwSALHandle_t sal,
                           dwContextHandle_t ctx,
                           dwCameraOutputType outputType)
    : m_sal(sal)
    , m_pendingFrame(nullptr)
    , m_started(false)
{
    dwImageProperties imageProperties;
    createSensor(imageProperties, params, outputType);

    m_framePipeline.reset(new CameraFramePipeline(imageProperties, outputType, ctx));
}

SimpleCamera::SimpleCamera(const dwImageProperties &outputProperties,
                           const dwSensorParams &params,
                           dwSALHandle_t sal,
                           dwContextHandle_t ctx,
                           dwCameraOutputType outputType)
    : SimpleCamera(params, sal, ctx, outputType)
{
    m_framePipeline->setOutputProperties(outputProperties);
}

SimpleCamera::SimpleCamera(const dwSensorParams &params,
                           dwSALHandle_t sal,
                           dwContextHandle_t ctx,
                           cudaStream_t stream,
                           dwCameraOutputType outputType,
                           dwSoftISPDemosaicMethod demosaicMethod,
                           bool localToneMapping)
    : m_sal(sal)
    , m_pendingFrame(nullptr)
    , m_started(false)
{
    dwImageProperties imageProperties;
    createSensor(imageProperties, params, DW_CAMERA_OUTPUT_NATIVE_RAW);

    dwSoftISPParams softISPParams;
    CHECK_DW_ERROR(dwSoftISP_initParamsFromCamera(&softISPParams, &m_cameraProperties));

    m_framePipeline.reset(new RawCameraFramePipeline(imageProperties,
                                                     m_cameraProperties.cameraType,
                                                     outputType,
                                                     stream,
                                                     softISPParams,
                                                     demosaicMethod,
                                                     localToneMapping,
                                                     ctx));
}

SimpleCamera::~SimpleCamera()
{
    if(m_pendingFrame)
        releaseFrame();

    if(m_sensor) {
        if (m_started)
            dwSensor_stop(m_sensor);
        dwSAL_releaseSensor(m_sensor);
    }
}

void SimpleCamera::createSensor(dwImageProperties &imageProps, const dwSensorParams &params, dwCameraOutputType outputType)
{
    CHECK_DW_ERROR( dwSAL_createSensor(&m_sensor, params, m_sal) );
    CHECK_DW_ERROR( dwSensorCamera_getSensorProperties(&m_cameraProperties, m_sensor) );
    CHECK_DW_ERROR( dwSensorCamera_getImageProperties(&imageProps, outputType, m_sensor) );

    log("SimpleCamera: Camera image: %ux%u\n", imageProps.width, imageProps.height);
}

void SimpleCamera::setOutputProperties(const dwImageProperties &outputProperties)
{
    m_framePipeline->setOutputProperties(outputProperties);
}

dwImageHandle_t SimpleCamera::readFrame()
{
    if(!m_started)
    {
        CHECK_DW_ERROR( dwSensor_start(m_sensor));
        m_started = true;
    }

    if(m_pendingFrame)
        releaseFrame();

    dwStatus status = dwSensorCamera_readFrame(&m_pendingFrame, 0, 1000000, m_sensor);

    if (status == DW_END_OF_STREAM) {
        log("SimpleCamera: Camera reached end of stream.\n");
        return nullptr;
    } else if (status == DW_NOT_READY) {
        while (status == DW_NOT_READY) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            status = dwSensorCamera_readFrame(&m_pendingFrame, 0, 1000000, m_sensor);
        }
    } else if (status != DW_SUCCESS) {
        throw std::runtime_error("Error reading from camera");
    }

    m_framePipeline->processFrame(m_pendingFrame);

    return m_framePipeline->getFrame();
}

bool SimpleCamera::enableSeeking(size_t &frameCount, dwTime_t &startTimestamp, dwTime_t &endTimestamp)
{
    // Try to get seek range of a sensor
    dwStatus res = dwSensor_getSeekRange(&frameCount, &startTimestamp, &endTimestamp, m_sensor);
    if(res != DW_SUCCESS)
    {
        // Seek table has not been created. Trying to create here.
        res = dwSensor_createSeekTable(m_sensor);
        if(res != DW_SUCCESS)
        {
            logError("SimpleCamera: Error creating index table: %s. Seeking is not available.\n", dwGetStatusName(res));
            return false;
        }

        CHECK_DW_ERROR_MSG(dwSensor_getSeekRange(&frameCount, &startTimestamp, &endTimestamp, m_sensor),
                           "Cannot obtain seek range from the camera.");
    }

    return true;
}

void SimpleCamera::seekToTime(dwTime_t timestamp)
{
    dwStatus res = dwSensor_seekToTime(timestamp, m_sensor);

    if(res != DW_SUCCESS) {
        logError("SimpleCamera: seek to time failed with %s.\n", dwGetStatusName(res));
    }
}

void SimpleCamera::seekToFrame(size_t frameIdx)
{
    dwStatus res = dwSensor_seekToEvent(frameIdx, m_sensor);

    if(res != DW_SUCCESS) {
        logError("SimpleCamera: seek to frame failed with %s.\n", dwGetStatusName(res));
    }
}

void SimpleCamera::releaseFrame()
{
    if(m_pendingFrame) {
        CHECK_DW_ERROR(dwSensorCamera_returnFrame(&m_pendingFrame));
        m_pendingFrame = nullptr;
    }
}

void SimpleCamera::resetCamera()
{
    CHECK_DW_ERROR(dwSensor_reset(m_sensor));
}

void SimpleCamera::enableGLOutput()
{
    m_framePipeline->enableGLOutput();
}

//////////////////////////////////////////////////////////////////////////////////////
/// RawSimpleCamera
///

RawSimpleCamera::RawSimpleCamera(const dwSensorParams &params, dwSALHandle_t sal, dwContextHandle_t ctx, cudaStream_t stream, 
        dwCameraOutputType outputType, dwSoftISPDemosaicMethod demosaicMethod, bool localToneMapping)
    : SimpleCamera(params, sal, ctx, stream, outputType, demosaicMethod, localToneMapping)
{
}

RawSimpleCamera::RawSimpleCamera(const dwImageFormat &outputISPFormat,
        const dwSensorParams &params, dwSALHandle_t sal, dwContextHandle_t ctx, cudaStream_t stream,
        dwCameraOutputType outputType, dwSoftISPDemosaicMethod demosaicMethod)
    : RawSimpleCamera(params, sal, ctx, stream, outputType, demosaicMethod)
{
    RawCameraFramePipeline* cameraIO = dynamic_cast<RawCameraFramePipeline*>(m_framePipeline.get());
    cameraIO->setOutputISPFormat(outputISPFormat);
}

RawSimpleCamera::RawSimpleCamera(const dwImageProperties &outputProperties, 
        const dwSensorParams &params, dwSALHandle_t sal, dwContextHandle_t ctx, cudaStream_t stream,
        dwCameraOutputType outputType, dwSoftISPDemosaicMethod demosaicMethod)
    : RawSimpleCamera(params, sal, ctx, stream, outputType, demosaicMethod)
{
    RawCameraFramePipeline* cameraIO = dynamic_cast<RawCameraFramePipeline*>(m_framePipeline.get());
    cameraIO->setRawOutputProperties(outputProperties);
}

}
}
