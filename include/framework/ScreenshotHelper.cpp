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

#include "ScreenshotHelper.hpp"

namespace dw_samples
{
namespace common
{

ScreenshotHelper::ScreenshotHelper(dwContextHandle_t ctx, dwSALHandle_t sal, uint32_t width, uint32_t height, std::string path)
{
    m_roi.x = 0;
    m_roi.y = 0;
    m_roi.width = width;
    m_roi.height = height;

    m_pathName = path;

    dwImageProperties properties{};
    properties.width = width;
    properties.height = height;
    properties.type = DW_IMAGE_GL;
    properties.format = DW_IMAGE_FORMAT_RGBA_UINT8;

    CHECK_DW_ERROR(dwImageStreamer_initialize(&m_streamer, &properties, DW_IMAGE_CPU, ctx));

    dwFrameCaptureParams frameParams{};

    frameParams.params.parameters = "";
    frameParams.width = width;
    frameParams.height = height;
    frameParams.mode = DW_FRAMECAPTURE_MODE_SCREENCAP;

    CHECK_DW_ERROR(dwFrameCapture_initialize(&m_frameCapture, &frameParams, sal, ctx));

    dwImageGL* imgGL;
    CHECK_DW_ERROR(dwFrameCapture_screenCapture(const_cast<const dwImageGL **>(&imgGL), m_roi, m_frameCapture));
    CHECK_DW_ERROR(dwImage_createAndBindGLTexture(&m_imageGL, imgGL->prop, imgGL->tex, imgGL->target));

    m_screenshotTrigger = false;
}

ScreenshotHelper::~ScreenshotHelper()
{
    if (m_imageGL) {
        dwImage_destroy(m_imageGL);
    }

    if (m_streamer) {
        dwImageStreamer_release(m_streamer);
    }

    if (m_frameCapture) {
        dwFrameCapture_release(m_frameCapture);
    }
}


void ScreenshotHelper::triggerScreenshot()
{
    m_screenshotTrigger = true;
}

void ScreenshotHelper::processScreenshotTrig()
{
    if(m_screenshotTrigger) {
        takeScreenshot();
        m_screenshotTrigger = false;
    }
}

void ScreenshotHelper::takeScreenshot()
{
    dwImageGL *imgGL;
    dwImage_getGL(&imgGL, m_imageGL);

    dwImageGL *capturedGL;
    CHECK_DW_ERROR(
            dwFrameCapture_screenCapture(const_cast<const dwImageGL **>(&capturedGL), m_roi, m_frameCapture));

    imgGL->tex = capturedGL->tex;

    dwImageHandle_t imageCPU;

    CHECK_DW_ERROR(dwImageStreamer_producerSend(m_imageGL, m_streamer));
    CHECK_DW_ERROR(dwImageStreamer_consumerReceive(&imageCPU, 33000, m_streamer));

    dwImageCPU *imgCPU;
    dwImage_getCPU(&imgCPU, imageCPU);

    char fname[128];
    sprintf(fname, "%s_screenshot_%04d.png", m_pathName.c_str(), m_screenshotCount++);
    lodepng_encode32_file(fname, imgCPU->data[0], m_roi.width, m_roi.height);
    std::cout << "SCREENSHOT TAKEN to " << fname << "\n";

    CHECK_DW_ERROR(dwImageStreamer_consumerReturn(&imageCPU, m_streamer));
    CHECK_DW_ERROR(dwImageStreamer_producerReturn(nullptr, 33000, m_streamer));
}

}
}

