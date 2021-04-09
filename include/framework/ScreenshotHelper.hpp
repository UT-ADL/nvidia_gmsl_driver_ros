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

#ifndef COMMON_SCREENSHOT_HPP_
#define COMMON_SCREENSHOT_HPP_


// Common
#include <framework/Checks.hpp>
// Driveworks
#include <dw/image/Image.h>
#include <dw/image/ImageStreamer.h>
#include <dw/image/FrameCapture.h>

#include <string>
#include <lodepng.h>

namespace dw_samples
{
namespace common
{

class ScreenshotHelper
{
  public:
    ScreenshotHelper(dwContextHandle_t ctx, dwSALHandle_t sal, uint32_t width, uint32_t height, std::string path);
    virtual ~ScreenshotHelper();

    // Set a flag to take a screenshot when processScreenshotTrig is called.
    // Useful for calling screenshot from different thread
    void triggerScreenshot();

    // If the flag is set, take a screenshot. This allows the screenshot to be called from a different thread.
    void processScreenshotTrig();

    // Take a screenshot. This must be called from a thread with access to GL context.
    void takeScreenshot();

private :
    dwImageStreamerHandle_t m_streamer;
    dwFrameCaptureHandle_t m_frameCapture;

    dwImageHandle_t m_imageGL;

    uint32_t m_screenshotCount;

    dwRect m_roi;

    std::string m_pathName;

    // internal flag to trigger a screenshot. Used by triggerScreenshot and processScreenshotTrig.
    bool m_screenshotTrigger;
};
}
}

#endif
