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

#ifndef SAMPLES_COMMON_WINDOW_HPP_
#define SAMPLES_COMMON_WINDOW_HPP_

#include <dw/core/EGL.h>
#include <dw/gl/GL.h>

#ifndef DW_USE_EGL
#include <GLFW/glfw3.h>
#endif

class WindowBase
{
  public:

    typedef void (*KeyDownCallback)(int key, int scancode, int mods);
    typedef void (*KeyUpCallback)(int key, int scancode, int mods);
    typedef void (*KeyRepeatCallback)(int key, int scancode, int mods);
    typedef void (*KeyCallback)(int key, int scancode, int action, int mods);
    typedef void (*MouseDownCallback)(int button, float x, float y, int mods);
    typedef void (*MouseUpCallback)(int button, float x, float y, int mods);
    typedef void (*MouseMoveCallback)(float x, float y);
    typedef void (*MouseWheelCallback)(float dx, float dy);
    typedef void (*CharModsCallback)(uint32_t codepoint, int32_t mods);
    typedef void (*ResizeWindowCallback)(int width, int height);

    // Factory
    static WindowBase *create(const char *title, int windowWidth,
            int windowHeight, bool offscreen, int samples = 0,
            bool initInvisible = false);
    static WindowBase *create(int windowWidth, int windowHeight,
            bool offscreen, int samples = 0);

    // create an X11 window
    //   width: width of window
    //   height: height of window
    WindowBase(int windowWidth, int windowHeight)
        : m_width(windowWidth)
        , m_height(windowHeight)
        , m_keyDownCallback(nullptr)
        , m_keyUpCallback(nullptr)
        , m_keyRepeatCallback(nullptr)
        , m_keyCallback(nullptr)
        , m_mouseDownCallback(nullptr)
        , m_mouseUpCallback(nullptr)
        , m_mouseMoveCallback(nullptr)
        , m_mouseWheelCallback(nullptr)
        , m_charModsCallback(nullptr)
        , m_resizeWindowCallback(nullptr)
    {
    }

    // release window
    virtual ~WindowBase()
    {
    }

    // swap back and front buffers
    virtual bool swapBuffers() = 0;

    // release EGL context
    virtual bool releaseContext() = 0;

    // reset EGL context
    virtual void resetContext() = 0;

    // create shared window context for the calling thread
#ifdef DW_USE_EGL
    virtual EGLContext createSharedContext() const = 0;
#else
    virtual GLFWwindow* createSharedContext() const = 0;
#endif

    // make window context current to the calling thread
    virtual bool makeCurrent() = 0;

    // remove current window context from the calling thread
    virtual bool resetCurrent() = 0;

    // indicate that a window should be closed, i.e. requested by the user
    virtual bool shouldClose() { return false; }

    // Set the window size
    virtual bool setWindowSize(int w, int h) { (void)w; (void)h; return false; }

    // Get the current desktop resolution
    virtual bool getDesktopResolution(int& w, int& h) { w = 1280; h=800; return false; }

    // Set windowed mode window to full screen
    virtual bool setFullScreen() { return false; }

    // indicate that the window is offscreen, hence no rendering needed
    virtual bool isOffscreen() const { return false; }

    // Set window position to center of screen
    virtual bool setWindowPosCentered() { return false; }

    virtual bool setWindowVisibility(bool visible) { (void)visible; return false; }

    // get EGL display
    virtual EGLDisplay getEGLDisplay(void) = 0;
    virtual EGLContext getEGLContext(void) = 0;

    int width() const { return m_width; }
    int height() const { return m_height; }

    void setOnKeyDownCallback(KeyDownCallback callback)
    {
        m_keyDownCallback = callback;
    }

    void setOnKeyUpCallback(KeyUpCallback callback)
    {
        m_keyUpCallback = callback;
    }

    void setOnKeyRepeatCallback(KeyRepeatCallback callback)
    {
        m_keyRepeatCallback = callback;
    }

    void setOnKeypressCallback(KeyCallback callback)
    {
        m_keyCallback = callback;
    }

    void setOnMouseDownCallback(MouseDownCallback callback)
    {
        m_mouseDownCallback = callback;
    }

    void setOnMouseUpCallback(MouseUpCallback callback)
    {
        m_mouseUpCallback = callback;
    }

    void setOnMouseMoveCallback(MouseMoveCallback callback)
    {
        m_mouseMoveCallback = callback;
    }

    void setOnMouseWheelCallback(MouseWheelCallback callback)
    {
        m_mouseWheelCallback = callback;
    }

    void setOnCharModsCallback(CharModsCallback callback)
    {
        m_charModsCallback = callback;
    }

    void setOnResizeWindowCallback(ResizeWindowCallback callback)
    {
        m_resizeWindowCallback = callback;
    }

  protected:
    int m_width;
    int m_height;

    KeyDownCallback m_keyDownCallback;
    KeyUpCallback m_keyUpCallback;
    KeyRepeatCallback m_keyRepeatCallback;
    KeyCallback m_keyCallback;
    MouseDownCallback m_mouseDownCallback;
    MouseUpCallback m_mouseUpCallback;
    MouseMoveCallback m_mouseMoveCallback;
    MouseWheelCallback m_mouseWheelCallback;
    CharModsCallback m_charModsCallback;
    ResizeWindowCallback m_resizeWindowCallback;
};

#endif // SAMPLES_COMMON_WINDOW_HPP_
