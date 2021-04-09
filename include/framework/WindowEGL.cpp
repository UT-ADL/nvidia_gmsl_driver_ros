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
// Copyright (c) 2014-2016 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#include "WindowEGL.hpp"

#if VIBRANTE_V5Q
#include "WindowQNXEGL.hpp"
#else
#include "WindowLinuxEGL.hpp"
#endif

#include <iostream>
#include <iomanip>
#include <stdexcept>

// -----------------------------------------------------------------------------
WindowEGL *WindowEGL::create(int32_t width, int32_t height, bool offscreen,
                             int32_t samples)
{
#if VIBRANTE_V5Q
    return (new WindowQNXEGL(width, height, offscreen, samples));
#else
    return (new WindowLinuxEGL(width, height, offscreen, samples));
#endif
}

// -----------------------------------------------------------------------------
WindowEGL *WindowEGL::create(int32_t width, int32_t height, bool offscreen)
{
    return create(width, height, offscreen, 0);
}

// -----------------------------------------------------------------------------
WindowEGL::WindowEGL(int32_t width, int32_t height, bool offscreen)
    : WindowBase(width, height)
    , m_display(EGL_NO_DISPLAY)
    , m_context(EGL_NO_CONTEXT)
    , m_surface(EGL_NO_SURFACE)
    , m_stream(EGL_NO_STREAM_KHR)
    , m_offscreen(offscreen)
{

    // Load Standard EGL Functions
    eglCreateStreamKHR = (PFNEGLCREATESTREAMKHRPROC)
                                    eglGetProcAddress("eglCreateStreamKHR");
    eglDestroyStreamKHR = (PFNEGLDESTROYSTREAMKHRPROC)
                                    eglGetProcAddress("eglDestroyStreamKHR");
    eglCreateStreamProducerSurfaceKHR = (PFNEGLCREATESTREAMPRODUCERSURFACEKHRPROC)
                                    eglGetProcAddress("eglCreateStreamProducerSurfaceKHR");

    // Load EGL Extensions
    eglGetPlatformDisplayEXT = (PFNEGLGETPLATFORMDISPLAYEXTPROC)
                                    eglGetProcAddress("eglGetPlatformDisplayEXT");
    eglCreatePlatformWindowSurfaceEXT = (PFNEGLCREATEPLATFORMWINDOWSURFACEEXTPROC)
                                    eglGetProcAddress("eglCreatePlatformWindowSurfaceEXT");
    eglCreatePlatformPixmapSurfaceEXT = (PFNEGLCREATEPLATFORMPIXMAPSURFACEEXTPROC)
                                    eglGetProcAddress("eglCreatePlatformPixmapSurfaceEXT");
    eglQueryDevicesEXT = (PFNEGLQUERYDEVICESEXTPROC)
                                    eglGetProcAddress("eglQueryDevicesEXT");
    eglQueryDeviceStringEXT = (PFNEGLQUERYDEVICESTRINGEXTPROC)
                                    eglGetProcAddress("eglQueryDeviceStringEXT");

    if (!eglCreateStreamKHR ||
            !eglDestroyStreamKHR ||
            !eglCreateStreamProducerSurfaceKHR ||
            !eglGetPlatformDisplayEXT ||
            !eglCreatePlatformWindowSurfaceEXT ||
            !eglCreatePlatformPixmapSurfaceEXT ||
            !eglQueryDevicesEXT ||
            !eglQueryDeviceStringEXT) {

        throw std::runtime_error("WindowEGL: Cannot load EGL extensions");
    }
}

// -----------------------------------------------------------------------------
WindowEGL::~WindowEGL()
{
    if (m_stream != EGL_NO_STREAM_KHR)
        eglDestroyStreamKHR(m_display, m_stream);

    if (m_surface != EGL_NO_SURFACE)
        eglDestroySurface(m_display, m_surface);

    eglDestroyContext(m_display, m_context);
    eglTerminate(m_display);
    eglReleaseThread();
}

// -----------------------------------------------------------------------------
bool WindowEGL::releaseContext()
{
    return eglMakeCurrent(m_display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
}

// -----------------------------------------------------------------------------
void WindowEGL::resetContext()
{
    eglDestroyContext(m_display, m_context);

    std::cout << "WindowEGL: create EGL context" << std::endl;
    {
        EGLint ctxAttribs[] = {
            EGL_CONTEXT_CLIENT_VERSION, 3,
            EGL_CONTEXT_OPENGL_ROBUST_ACCESS_EXT, EGL_FALSE,
            EGL_CONTEXT_OPENGL_RESET_NOTIFICATION_STRATEGY_EXT, EGL_NO_RESET_NOTIFICATION_EXT,
            EGL_NONE, EGL_NONE};
        m_context = eglCreateContext(m_display, m_config, EGL_NO_CONTEXT, ctxAttribs);
        if (m_context == EGL_NO_CONTEXT) {
            std::cout << "WindowEGL: Failed to create EGL context" << std::endl;
            throw std::exception();
        }
    }

    std::cout << "WindowEGL: assign EGL context to current thread" << std::endl;
    {
        EGLBoolean status = eglMakeCurrent(m_display, m_surface, m_surface, m_context);
        if (status == EGL_FALSE) {
            std::cout << "WindowEGL: Could not makeCurrent: "
                      << std::hex << eglGetError() << std::dec << std::endl;
            throw std::exception();
        }
    }
}

// -----------------------------------------------------------------------------
EGLContext WindowEGL::createSharedContext() const
{
    std::cout << "WindowEGL: create shared EGL context" << std::endl;

    EGLint ctxAttribs[] = {
        EGL_CONTEXT_CLIENT_VERSION, 3,
        EGL_CONTEXT_OPENGL_ROBUST_ACCESS_EXT, EGL_FALSE,
        EGL_CONTEXT_OPENGL_RESET_NOTIFICATION_STRATEGY_EXT, EGL_NO_RESET_NOTIFICATION_EXT,
        EGL_NONE, EGL_NONE};

    EGLContext shared = eglCreateContext(m_display, m_config, m_context, ctxAttribs);

    if (shared == EGL_NO_CONTEXT) {
        std::cout << "WindowEGL: Failed to create shared EGL context" << eglGetError() << std::endl;
        throw std::exception();
    }

    EGLBoolean status = eglMakeCurrent(m_display, EGL_NO_SURFACE, EGL_NO_SURFACE, shared);
    if (status != EGL_TRUE) {
        std::cout << "WindowEGL: Failed to make shared EGL context current: " << eglGetError() << std::endl;
        throw std::exception();
    }

    return shared;
}

// -----------------------------------------------------------------------------
bool WindowEGL::makeCurrent()
{
    return eglMakeCurrent(m_display, m_surface, m_surface, m_context) == EGL_TRUE;
}

// -----------------------------------------------------------------------------
bool WindowEGL::resetCurrent()
{
    return eglMakeCurrent(m_display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT) == EGL_TRUE;
}

// -----------------------------------------------------------------------------
bool WindowEGL::swapBuffers()
{
    return eglSwapBuffers(m_display, m_surface) == EGL_TRUE;
}
