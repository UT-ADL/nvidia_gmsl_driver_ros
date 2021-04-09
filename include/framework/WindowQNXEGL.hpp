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

#ifndef SAMPLES_COMMON_WINDOW_QNX_EGL_HPP_
#define SAMPLES_COMMON_WINDOW_QNX_EGL_HPP_

#include <thread>
#include <atomic>
#include <map>

#include <EGL/egl.h>
#include <EGL/eglext.h>

#include "WindowEGL.hpp"

#include <screen/screen.h>

/**
 * @brief Enumeration of keycode values reported by screen
 */
enum QNXScreenKeycode : int32_t {
    QNX_SCREEN_KEY_FIRST          = 96,
    QNX_SCREEN_KEY_A              = 97,
    QNX_SCREEN_KEY_B              = 98,
    QNX_SCREEN_KEY_C              = 99,
    QNX_SCREEN_KEY_D              = 100,
    QNX_SCREEN_KEY_E              = 101,
    QNX_SCREEN_KEY_F              = 102,
    QNX_SCREEN_KEY_G              = 103,
    QNX_SCREEN_KEY_H              = 104,
    QNX_SCREEN_KEY_I              = 105,
    QNX_SCREEN_KEY_J              = 106,
    QNX_SCREEN_KEY_K              = 107,
    QNX_SCREEN_KEY_L              = 108,
    QNX_SCREEN_KEY_M              = 109,
    QNX_SCREEN_KEY_N              = 110,
    QNX_SCREEN_KEY_O              = 111,
    QNX_SCREEN_KEY_P              = 112,
    QNX_SCREEN_KEY_Q              = 113,
    QNX_SCREEN_KEY_R              = 114,
    QNX_SCREEN_KEY_S              = 115,
    QNX_SCREEN_KEY_T              = 116,
    QNX_SCREEN_KEY_U              = 117,
    QNX_SCREEN_KEY_V              = 118,
    QNX_SCREEN_KEY_W              = 119,
    QNX_SCREEN_KEY_X              = 120,
    QNX_SCREEN_KEY_Y              = 121,
    QNX_SCREEN_KEY_Z              = 122,
    QNX_SCREEN_KEY_BACKSPACE      = 61448,
    QNX_SCREEN_KEY_TAB            = 61449,
    QNX_SCREEN_KEY_ENTER          = 61453,
    QNX_SCREEN_KEY_SCROLL_LOCK    = 61460,
    QNX_SCREEN_KEY_ESCAPE         = 61467,
    QNX_SCREEN_KEY_HOME           = 61520,
    QNX_SCREEN_KEY_LEFT           = 61521,
    QNX_SCREEN_KEY_UP             = 61522,
    QNX_SCREEN_KEY_RIGHT          = 61523,
    QNX_SCREEN_KEY_DOWN           = 61524,
    QNX_SCREEN_KEY_PAGE_UP        = 61525,
    QNX_SCREEN_KEY_PAGE_DOWN      = 61526,
    QNX_SCREEN_KEY_END            = 61527,
    QNX_SCREEN_KEY_INSERT         = 61539,
    QNX_SCREEN_KEY_NUM_LOCK       = 61567,
    QNX_SCREEN_KEY_DELETE         = 61695,
    QNX_SCREEN_KEY_F1             = 61630,
    QNX_SCREEN_KEY_F2             = 61631,
    QNX_SCREEN_KEY_F3             = 61632,
    QNX_SCREEN_KEY_F4             = 61633,
    QNX_SCREEN_KEY_F5             = 61634,
    QNX_SCREEN_KEY_F6             = 61635,
    QNX_SCREEN_KEY_F7             = 61636,
    QNX_SCREEN_KEY_F8             = 61637,
    QNX_SCREEN_KEY_F9             = 61638,
    QNX_SCREEN_KEY_F10            = 61639,
    QNX_SCREEN_KEY_F11            = 61640,
    QNX_SCREEN_KEY_F12            = 61641,
    QNX_SCREEN_KEY_LEFT_SHIFT     = 61665,
    QNX_SCREEN_KEY_RIGHT_SHIFT    = 61666,
    QNX_SCREEN_KEY_LEFT_CONTROL   = 61667,
    QNX_SCREEN_KEY_RIGHT_CONTROL  = 61668,
    QNX_SCREEN_KEY_CAPS_LOCK      = 61669,
    QNX_SCREEN_KEY_LEFT_ALT       = 61673,
    QNX_SCREEN_KEY_RIGHT_ALT      = 61674,
    QNX_SCREEN_KEY_LAST,
};

/**
 * @brief The EGLDisplay class on QNX platform
 */
class WindowQNXEGL : public WindowEGL
{
  public:
    WindowQNXEGL(int32_t width, int32_t height, bool offscreen, int32_t samples);
    virtual ~WindowQNXEGL();
    bool getDesktopResolution(int& width, int& height) override;

  private:
    void windowSystemInit(EGLint displayId,
                        EGLDisplay &eglDisplay);
    void windowSystemWindowInit(EGLint width,
                        EGLint height,
                        EGLint xoffset,
                        EGLint yoffset,
                        EGLint windowId,
                        EGLDisplay display,
                        EGLConfig config,
                        EGLint &displayId);
    void windowSystemEglSurfaceCreate(EGLDisplay display,
                        EGLConfig config,
                        EGLSurface &surface);
    int windowSystemChooseFormat(void *disp,
                        void *egl_conf);
    void windowSystemTerminate(void);
    void windowSystemWindowTerminate();
    int getDisplayNumber();

    static void windowSystemEventLoop(WindowQNXEGL *window);
    static int getGLFWKeycode(QNXScreenKeycode key);

    screen_context_t m_screenCtx = nullptr;
    screen_display_t m_screenDisplay = nullptr;
    screen_window_t m_screenWindow = nullptr;

    std::thread m_eventThread;
    std::atomic<bool> m_eventThreadRun;

    static std::map<QNXScreenKeycode, int> screenKeyToGLFW;
};

#endif // SAMPLES_COMMON_WINDOW_QNX_EGL_HPP_
