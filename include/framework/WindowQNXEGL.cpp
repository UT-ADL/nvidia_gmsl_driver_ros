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
// Copyright (c) 2017 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#include "WindowGLFW.hpp"
#include "WindowQNXEGL.hpp"

#include <EGL/eglext.h>

#include <iostream>
#include <iomanip>
#include <stdexcept>

// -----------------------------------------------------------------------------
std::map<QNXScreenKeycode, int> WindowQNXEGL::screenKeyToGLFW = {
    {QNX_SCREEN_KEY_A,             GLFW_KEY_A},
    {QNX_SCREEN_KEY_B,             GLFW_KEY_B},
    {QNX_SCREEN_KEY_C,             GLFW_KEY_C},
    {QNX_SCREEN_KEY_D,             GLFW_KEY_D},
    {QNX_SCREEN_KEY_E,             GLFW_KEY_E},
    {QNX_SCREEN_KEY_F,             GLFW_KEY_F},
    {QNX_SCREEN_KEY_G,             GLFW_KEY_G},
    {QNX_SCREEN_KEY_H,             GLFW_KEY_H},
    {QNX_SCREEN_KEY_I,             GLFW_KEY_I},
    {QNX_SCREEN_KEY_J,             GLFW_KEY_J},
    {QNX_SCREEN_KEY_K,             GLFW_KEY_K},
    {QNX_SCREEN_KEY_L,             GLFW_KEY_L},
    {QNX_SCREEN_KEY_M,             GLFW_KEY_M},
    {QNX_SCREEN_KEY_N,             GLFW_KEY_N},
    {QNX_SCREEN_KEY_O,             GLFW_KEY_O},
    {QNX_SCREEN_KEY_P,             GLFW_KEY_P},
    {QNX_SCREEN_KEY_Q,             GLFW_KEY_Q},
    {QNX_SCREEN_KEY_R,             GLFW_KEY_R},
    {QNX_SCREEN_KEY_S,             GLFW_KEY_S},
    {QNX_SCREEN_KEY_T,             GLFW_KEY_T},
    {QNX_SCREEN_KEY_U,             GLFW_KEY_U},
    {QNX_SCREEN_KEY_V,             GLFW_KEY_V},
    {QNX_SCREEN_KEY_W,             GLFW_KEY_W},
    {QNX_SCREEN_KEY_X,             GLFW_KEY_X},
    {QNX_SCREEN_KEY_Y,             GLFW_KEY_Y},
    {QNX_SCREEN_KEY_Z,             GLFW_KEY_Z},
    {QNX_SCREEN_KEY_BACKSPACE,     GLFW_KEY_BACKSPACE},
    {QNX_SCREEN_KEY_TAB,           GLFW_KEY_TAB},
    {QNX_SCREEN_KEY_ENTER,         GLFW_KEY_ENTER},
    {QNX_SCREEN_KEY_ESCAPE,        GLFW_KEY_ESCAPE},
    {QNX_SCREEN_KEY_HOME,          GLFW_KEY_HOME},
    {QNX_SCREEN_KEY_LEFT,          GLFW_KEY_LEFT},
    {QNX_SCREEN_KEY_UP,            GLFW_KEY_UP},
    {QNX_SCREEN_KEY_RIGHT,         GLFW_KEY_RIGHT},
    {QNX_SCREEN_KEY_DOWN,          GLFW_KEY_DOWN},
    {QNX_SCREEN_KEY_PAGE_UP,       GLFW_KEY_PAGE_UP},
    {QNX_SCREEN_KEY_PAGE_DOWN,     GLFW_KEY_PAGE_DOWN},
    {QNX_SCREEN_KEY_END,           GLFW_KEY_END},
    {QNX_SCREEN_KEY_INSERT,        GLFW_KEY_INSERT},
    {QNX_SCREEN_KEY_NUM_LOCK,      GLFW_KEY_NUM_LOCK},
    {QNX_SCREEN_KEY_DELETE,        GLFW_KEY_DELETE},
    {QNX_SCREEN_KEY_F1,            GLFW_KEY_F1},
    {QNX_SCREEN_KEY_F2,            GLFW_KEY_F2},
    {QNX_SCREEN_KEY_F3,            GLFW_KEY_F3},
    {QNX_SCREEN_KEY_F4,            GLFW_KEY_F4},
    {QNX_SCREEN_KEY_F5,            GLFW_KEY_F5},
    {QNX_SCREEN_KEY_F6,            GLFW_KEY_F6},
    {QNX_SCREEN_KEY_F7,            GLFW_KEY_F7},
    {QNX_SCREEN_KEY_F8,            GLFW_KEY_F8},
    {QNX_SCREEN_KEY_F9,            GLFW_KEY_F9},
    {QNX_SCREEN_KEY_F10,           GLFW_KEY_F10},
    {QNX_SCREEN_KEY_F11,           GLFW_KEY_F11},
    {QNX_SCREEN_KEY_F12,           GLFW_KEY_F12},
    {QNX_SCREEN_KEY_LEFT_SHIFT,    GLFW_KEY_LEFT_SHIFT},
    {QNX_SCREEN_KEY_RIGHT_SHIFT,   GLFW_KEY_RIGHT_SHIFT},
    {QNX_SCREEN_KEY_LEFT_CONTROL,  GLFW_KEY_LEFT_CONTROL},
    {QNX_SCREEN_KEY_RIGHT_CONTROL, GLFW_KEY_RIGHT_CONTROL},
    {QNX_SCREEN_KEY_CAPS_LOCK,     GLFW_KEY_CAPS_LOCK},
    {QNX_SCREEN_KEY_LEFT_ALT,      GLFW_KEY_LEFT_ALT},
    {QNX_SCREEN_KEY_RIGHT_ALT,     GLFW_KEY_RIGHT_ALT}
};

// -----------------------------------------------------------------------------
WindowQNXEGL::WindowQNXEGL(int32_t width, int32_t height, bool offscreen, int32_t samples)
    : WindowEGL(width, height, offscreen)
{
    // Assume we are always using the 1st display
    EGLint displayId = getDisplayNumber();
    windowSystemInit(displayId, m_display);

    // EGL Init
    EGLBoolean status = eglInitialize(m_display, 0, 0);
    if (!status)
        throw std::runtime_error("WindowQNXEGL: could not initialize EGL");

    // Initialize the Window
    EGLint xoffset = 0, yoffset = 0;
    EGLint windowId = 0;
    windowSystemWindowInit(width, height, xoffset, yoffset, windowId,
            m_display, m_config, displayId);

    // Choose configuration
    EGLint cfgAttribs[] = {
        EGL_SURFACE_TYPE, EGL_STREAM_BIT_KHR,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT,
        EGL_RED_SIZE, 5,
        EGL_GREEN_SIZE, 6,
        EGL_BLUE_SIZE, 5,
        EGL_ALPHA_SIZE, 8,
        EGL_DEPTH_SIZE, 8,
        EGL_STENCIL_SIZE, 8,
        EGL_SAMPLE_BUFFERS, 1,
        EGL_SAMPLES, samples,
        EGL_NONE, EGL_NONE
    };

    // Query the # of matching configs
    EGLint cfgCount = 0;
    status = eglChooseConfig(m_display, cfgAttribs, nullptr, 0, &cfgCount);
    if (!status || cfgCount == 0)
        throw std::runtime_error("WindowQNXEGL: could not read EGL config count");

    // And, now read them
    EGLConfig *cfgs = new EGLConfig[cfgCount];
    status = eglChooseConfig(m_display, cfgAttribs, cfgs, cfgCount, &cfgCount);
    if (!status || cfgCount == 0)
        throw std::runtime_error("WindowQNXEGL: could not read EGL config");

    eglBindAPI(EGL_OPENGL_ES_API);

    // Take the first matching config
    m_config = cfgs[0];
    delete[] cfgs;

    // Create the EGL context
    EGLint ctxAttribs[] = {
        EGL_CONTEXT_CLIENT_VERSION, 3,
        EGL_CONTEXT_OPENGL_ROBUST_ACCESS_EXT, EGL_FALSE,
        EGL_CONTEXT_OPENGL_RESET_NOTIFICATION_STRATEGY_EXT, EGL_NO_RESET_NOTIFICATION_EXT,
        EGL_NONE, EGL_NONE
    };

    // Create EGL Stream
    EGLint stream_attr[] = {
        EGL_STREAM_FIFO_LENGTH_KHR, 1,
        EGL_NONE
    };
    m_stream = eglCreateStreamKHR(m_display, stream_attr);
    if (m_stream == EGL_NO_STREAM_KHR)
        throw std::runtime_error("WindowQNXEGL: could not read EGL stream");

    m_context = eglCreateContext(m_display, m_config, nullptr, ctxAttribs);
    if(m_context == EGL_NO_CONTEXT)
        throw std::runtime_error("WindowQNXEGL: could not read EGL context");

    // Create surface
    windowSystemEglSurfaceCreate(m_display, m_config, m_surface);

    status = eglQuerySurface(m_display, m_surface, EGL_WIDTH,  &width);
    status &= eglQuerySurface(m_display, m_surface, EGL_HEIGHT, &height);
    if (!status)
        throw std::runtime_error("WindowQNXEGL: could not query surface properties");

    // Attach the suface to the context
    status = eglMakeCurrent(m_display, m_surface, m_surface, m_context);
    if(!status)
        throw std::runtime_error("WindowQNXEGL: could not set egl context");

    status = eglSwapInterval(m_display, 0);
    if(!status)
        throw std::runtime_error("WindowQNXEGL: could not set vsync");

    // Start event loop
    m_eventThreadRun = true;
    m_eventThread = std::thread(windowSystemEventLoop, this);
}

// -----------------------------------------------------------------------------
WindowQNXEGL::~WindowQNXEGL()
{
    eglDestroySurface(m_display, m_surface);
    eglDestroyStreamKHR(m_display, m_stream);
    eglTerminate(m_display);

    // Sync w/ the event loop
    m_eventThreadRun = false;
    m_eventThread.join();

    windowSystemWindowTerminate();
    windowSystemTerminate();
}

//------------------------------------------------------------------------------
bool WindowQNXEGL::getDesktopResolution(int& width, int& height)
{
    int screen_resolution[2] = { 0, 0 };
    screen_get_display_property_iv(m_screenDisplay, SCREEN_PROPERTY_SIZE, screen_resolution);
    if((screen_resolution[0] == 0) && (screen_resolution[1] == 0)){
        width = 1920;
        height = 1080;
        return false;
    }
    else{
        width = screen_resolution[0];
        height = screen_resolution[1];
        return true;
    }
}

// -----------------------------------------------------------------------------
void WindowQNXEGL::windowSystemEventLoop(WindowQNXEGL *window)
{
    screen_event_t screenEv{};
    int ret = screen_create_event(&screenEv);
    if (ret < 0) {
        std::cerr << "WindowQNXEGL: unable to create screen event"
                  << std::endl;
    }

    while (window->m_eventThreadRun) {
        ret = screen_get_event(window->m_screenCtx, screenEv, 1000000000);
        if (ret < 0) {
            std::cerr << "WindowQNXEGL: screen_get_event failure "
                      << "(" << ret << ")"
                      << std::endl;
            break;
        }

        int type;
        screen_get_event_property_iv(screenEv, SCREEN_PROPERTY_TYPE, &type);
        if (type == SCREEN_EVENT_KEYBOARD) {
            int scanCode, keyCode, flags;
            screen_get_event_property_iv(screenEv, SCREEN_PROPERTY_SCAN, &scanCode);
            screen_get_event_property_iv(screenEv, SCREEN_PROPERTY_KEY_CAP, &keyCode);
            screen_get_event_property_iv(screenEv, SCREEN_PROPERTY_FLAGS, &flags);

            // Remap key to GLFW keycode values
            keyCode = getGLFWKeycode(static_cast<QNXScreenKeycode>(keyCode));

            if (flags & SCREEN_FLAG_KEY_REPEAT) {
                if (window->m_keyRepeatCallback)
                    window->m_keyRepeatCallback(keyCode, scanCode, 0);
            } else if (flags & SCREEN_FLAG_KEY_DOWN) {
                if (window->m_keyDownCallback)
                    window->m_keyDownCallback(keyCode, scanCode, 0);
                if (window->m_keyCallback)
                    window->m_keyCallback(keyCode, scanCode, GLFW_PRESS, 0);
            } else {
                if (window->m_keyUpCallback)
                    window->m_keyUpCallback(keyCode, scanCode, 0);
            }
        }
    }
}

// -----------------------------------------------------------------------------
int WindowQNXEGL::getGLFWKeycode(QNXScreenKeycode key)
{
    // Keys w/ a lower value than 'A' already align w/ GLFW keycodes
    if (key <= QNX_SCREEN_KEY_FIRST) {
        return key;
    } else {
        if (screenKeyToGLFW.find(key) == screenKeyToGLFW.end())
            return GLFW_KEY_UNKNOWN;
        return screenKeyToGLFW[key];
    }
}

// -----------------------------------------------------------------------------
void WindowQNXEGL::windowSystemInit(EGLint displayId, EGLDisplay &eglDisplay)
{
    int displayCount = 0;
    int attached = 0;
    screen_display_t displayHandle[16]{};

    // Create Screen Context
    int ret = screen_create_context(&m_screenCtx, 0);
    if (ret)
        throw std::runtime_error("WindowQNXEGL: could not create screen context");

    // Set the requested display
    screen_get_context_property_iv(m_screenCtx, SCREEN_PROPERTY_DISPLAY_COUNT,
                            &displayCount);
    if (displayId >= displayCount)
        throw std::runtime_error("WindowQNXEGL: Failed to set requested display");

    screen_get_context_property_pv(m_screenCtx,
                                   SCREEN_PROPERTY_DISPLAYS,
                                   (void **) displayHandle);
    screen_get_display_property_iv(displayHandle[displayId],
                                   SCREEN_PROPERTY_ATTACHED,
                                   &attached);
    if (attached)
        m_screenDisplay = displayHandle[displayId];
    else
        throw std::runtime_error("WindowQNXEGL: Requested display not attached");

    eglDisplay = eglGetDisplay(EGL_DEFAULT_DISPLAY);
}

// -----------------------------------------------------------------------------
void WindowQNXEGL::windowSystemWindowInit(EGLint width, EGLint height, EGLint xoffset,
                                      EGLint yoffset, EGLint windowId, EGLDisplay display,
                                      EGLConfig config, EGLint &displayId)
{
    int usage = SCREEN_USAGE_OPENGL_ES2;
    int nbuffers = 2;
    int format;
    int tempSize[2];
    int pipelineId, defaultpipelineId = 0;
    const int NUM_PIPELINES = 3;
    int transparency = SCREEN_TRANSPARENCY_SOURCE;
    EGLint interval = 1;

    // Create a native window
    int ret = screen_create_window(&m_screenWindow, m_screenCtx);
    if (ret)
        throw std::runtime_error("WindowQNXEGL: Failed to create native window");

    // Set the requested display
    screen_set_window_property_pv(m_screenWindow, SCREEN_PROPERTY_DISPLAY,
            (void **) &m_screenDisplay);

    // Configure the requested layer.
    // Screen API does not have much support for querying the display pipeline information.
    // Hard coding the pipeline ID's and the number of pipelines supported per display for now.
    if (windowId < 0 || windowId >= NUM_PIPELINES)
        throw std::runtime_error("WindowQNXEGL: Failed to set requested layer");

    // Query default pipelineId from Screen compositor
    ret = screen_get_window_property_iv(m_screenWindow,
            SCREEN_PROPERTY_PIPELINE, &defaultpipelineId);
    if (ret) 
        throw std::runtime_error("WindowQNXEGL: Could not query default layer");

    // Each display can have maximum 10 pipeline ids
    displayId = defaultpipelineId / 10;
    pipelineId = (10 * displayId) + windowId + 1;
    ret = screen_set_window_property_iv(m_screenWindow,
            SCREEN_PROPERTY_PIPELINE, &pipelineId);
    if (ret)
        throw std::runtime_error("WindowQNXEGL: Could not set pipeline id");

    // Select and set window property SCREEN_PROPERTY_FORMAT
    format = windowSystemChooseFormat(display, config);
    ret = screen_set_window_property_iv(m_screenWindow,
            SCREEN_PROPERTY_FORMAT, &format);
    if (ret)
        throw std::runtime_error("WindowQNXEGL: Failed to set property "
                                 "SCREEN_PROPERTY_FORMAT");

    // defaultpipelineId used by QNX Screen compositor
    if (pipelineId != defaultpipelineId)
        usage |= SCREEN_USAGE_OVERLAY;

    // Set window property SCREEN_PROPERTY_USAGE
    ret = screen_set_window_property_iv(m_screenWindow,
            SCREEN_PROPERTY_USAGE, &usage);
    if (ret)
        throw std::runtime_error("WindowQNXEGL: Failed to set property "
                                 "SCREEN_PROPERTY_USAGE");

    // Set window property SCREEN_PROPERTY_SWAP_INTERVAL
    ret = screen_set_window_property_iv(m_screenWindow,
            SCREEN_PROPERTY_SWAP_INTERVAL, &interval);
    if (ret)
        throw std::runtime_error("WindowQNXEGL: Failed to set property "
                                 "SCREEN_PROPERTY_SWAP_INTERVAL");

    if (width > 0 && height > 0) {
        // Set window property SCREEN_PROPERTY_SIZE
        // For QNX, we set full screen according to desktop resolution
        getDesktopResolution(tempSize[0], tempSize[1]);
        m_width = tempSize[0];
        m_height = tempSize[1];

        ret = screen_set_window_property_iv(m_screenWindow,
                SCREEN_PROPERTY_SIZE, tempSize);
        if (ret)
            throw std::runtime_error("WindowQNXEGL: Failed to set property "
                                     "SCREEN_PROPERTY_SIZE");
    }

    if (xoffset != 0 && yoffset != 0) {
        // Set window property SCREEN_PROPERTY_POSTION
        tempSize[0] = xoffset;
        tempSize[1] = yoffset;
        ret = screen_set_window_property_iv(m_screenWindow,
                SCREEN_PROPERTY_POSITION, tempSize);
        if (ret)
            throw std::runtime_error("WindowQNXEGL: Failed to set property "
                                     "SCREEN_PROPERTY_POSITION");
    }

    ret = screen_set_window_property_iv(m_screenWindow,
            SCREEN_PROPERTY_TRANSPARENCY, &transparency);
    if (ret)
        throw std::runtime_error("WindowQNXEGL: Failed to set property "
                                 "SCREEN_PROPERTY_TRANSPARENCY");

    // Create Screen window buffers
    ret = screen_create_window_buffers(m_screenWindow, nbuffers);
    if (ret)
        throw std::runtime_error("WindowQNXEGL: Failed to create window buffers");
}

// -----------------------------------------------------------------------------
void WindowQNXEGL::windowSystemEglSurfaceCreate(EGLDisplay display,
        EGLConfig config, EGLSurface &surface)
{
    EGLint srfAttribs[] = {
        EGL_NONE, EGL_NONE
    };

    // Create EGL surface
    surface = eglCreateWindowSurface(display,
                                     config,
                                     (NativeWindowType)(m_screenWindow),
                                     srfAttribs);
    if (surface == EGL_NO_SURFACE)
        throw std::runtime_error("WindowQNXEGL: Failed to create window surface");
}

// -----------------------------------------------------------------------------
int WindowQNXEGL::windowSystemChooseFormat(void *disp, void *egl_conf)
{
    int ret;
    EGLint bufSize;

    EGLDisplay dpy = (EGLDisplay)(disp);
    EGLConfig cfg = (EGLConfig)(egl_conf);

    eglGetConfigAttrib(dpy, cfg, EGL_BUFFER_SIZE, &bufSize);

    switch (bufSize) {
        case 16:
            ret = SCREEN_FORMAT_RGB565;
        break;
        case 24:
            ret = SCREEN_FORMAT_RGBX8888;
        break;
        case 32:
        default:
            ret = SCREEN_FORMAT_RGBA8888;
        break;
    }

    return ret;
}

// -----------------------------------------------------------------------------
void WindowQNXEGL::windowSystemWindowTerminate(void)
{
    if (!m_screenCtx || !m_screenWindow)
        return;

    // Destroy native window & event
    screen_destroy_window(m_screenWindow);
    m_screenWindow = NULL;
}

// -----------------------------------------------------------------------------
void WindowQNXEGL::windowSystemTerminate(void)
{
    if (m_screenCtx)
        screen_destroy_context(m_screenCtx);

    m_screenCtx = NULL;
}

// -----------------------------------------------------------------------------
int WindowQNXEGL::getDisplayNumber()
{
    char *displayValue = getenv("DISPLAY");

    // Default to 0, if environment is not set
    if (!displayValue)
        return 0;

    return atoi(displayValue);
}
