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

#ifndef DRIVEWORKSSAMPLE_HPP_
#define DRIVEWORKSSAMPLE_HPP_

// C++ Std
#include <memory>
#include <chrono>
#include <vector>
#include <type_traits>

// Common
#include "Window.hpp"
#include "MouseView3D.hpp"
#include "ProgramArguments.hpp"
#include "ProfilerCUDA.hpp"
#include "Checks.hpp"
#include "Log.hpp"
#include <framework/DataPath.hpp>
#include "ScreenshotHelper.hpp"
#include "RenderUtils.hpp"

namespace dw_samples
{
namespace common
{

//-------------------------------------------------------------------------------
/**
* Base class for Driveworks sample applications.
* Provides basic UI functions.
* Provides basic loop supporting pause and resume.
*/
class DriveWorksSample
{
public:
    //! initialize application (default is console mode, no GL context)
    DriveWorksSample(const ProgramArguments& args);

    virtual ~DriveWorksSample() {}

    /**
     * Initialize window application. This will also create a valid GL context.
     *
     * @param windowTitle, width, height Properties for the created window.
     * @param offscreen If true window will not be created, but full GL initialization happens.
     *                  Rendering will happen into an offscreen window.
     * @param samples   Specifies the desired number of samples to use for subsampling
     **/
    void initializeWindow(const char* windowTitle, int width, int height, bool offscreen = false, int samples = 0);

    /**
     * @brief Run background thread to capture key input.
     */
    void initializeCommandLineInput();

    /**
    * Starts the main loop. Frame-rate is limited to whatever is specified by setProcessRate().
    * Derived classes can use the default loop by overwritting the onProcess() and onRender() methods.
    * During a normal loop, both onProcess() and onRender() are called for each iteration.
    * If the application is paused, only onRender() will be called.
    *
    * This function will execute onInitialize() before loop and onRelease() after the loop.
    * If onInitialize() returns false, it will call onRelease() as well.
    */
    virtual int run();

    // ------------------------   state of the application ----------------------------
    /**
    * returns true if the app still needs to run
    */
    bool shouldRun();

    /**
     * @brief resume Call to resume the app programatically.
     */
    void resume();

    /**
     * @brief pause Call to pause the app programatically.
     */
    void pause();

    /**
     * @brief isPaused Call to test if app is paused.
     */
    bool isPaused() const;

    /**
     * @brief reset Call to reset the app programatically.
     */
    void reset();

    /**
     * @brief stop Call to stop the app programatically.
     */
    void stop();

    /**
     * @brief setProcessRate This controls how fast the process callback is called.
     * @param loopsPerSecond The max number of times process should be called per
     *                       second. Default is 0 (no limit).
     * @note If loopsPerSecond is 0 then no limitation happens.
     */
    void setProcessRate(int loopsPerSecond);

    /**
     * @brief setRenderRate This controls how fast the render callback is called.
     * @param loopsPerSecond The max number of times render should be called per
     *                       second. Default is 60fps.
     * @note If loopsPerSecond is 0 then no limitation happens.
     */
    void setRenderRate(int loopsPerSecond);

    /**
     * @brief setStopFrame Sets the frame number indicating when to stop running the sample
     * @param stopFrame frame number on which the sample should stop
     * @note if stopFrame is 0 the sample keeps running indefinitely (or until other stop conditions are reached)
     */
    void setStopFrame(uint32_t stopFrame);

    // ------------------------   Callbacks ----------------------------
    /**
     * @brief onInitialize Called right before run loop. Override this to initialize any app state.
     *
     * If derived application returns false, then no application stops
     */
    virtual bool onInitialize() = 0;

    /**
     * @brief onProcess Called in the main loop and used for processing
     * app data. This is not called if the app is paused.
     */
    virtual void onProcess() = 0;

    /**
     * @brief onRelease Called after run loop is finished. Release application code here.
     */
    virtual void onRelease() = 0;

    /**
     * @brief onPause Called in the main loop while application is pausing.
     */
    virtual void onPause() {}

    /**
     * @brief onReset Called in the main loop if reset has been requested
     */
    virtual void onReset();

    /**
     * @brief onRender Called in the main loop even when paused.
     * The method is not executed for non-window applications.
     */
    virtual void onRender() {}

    /**
     * @brief onResize called on any event when window size changed
     */
    virtual void onResizeWindow(int width, int height)
    {
        (void)width;
        (void)height;
    }

    /// overload these methods to react on different events
    virtual void onKeyDown(int key, int scancode, int mods)
    {
        (void)key;
        (void)scancode;
        (void)mods;
    }
    virtual void onKeyUp(int key, int scancode, int mods)
    {
        (void)key;
        (void)scancode;
        (void)mods;
    }
    virtual void onKeyRepeat(int key, int scancode, int mods)
    {
        (void)key;
        (void)scancode;
        (void)mods;
    }
    virtual void onCharMods(unsigned int codepoint, int mods)
    {
        (void)codepoint;
        (void)mods;
    }
    virtual void onMouseDown(int button, float x, float y, int mods)
    {
        (void)button;
        (void)x;
        (void)y;
        (void)mods;
    }
    virtual void onMouseUp(int button, float x, float y, int mods)
    {
        (void)button;
        (void)x;
        (void)y;
        (void)mods;
    }
    virtual void onMouseMove(float x, float y)
    {
        (void)x;
        (void)y;
    }
    virtual void onMouseWheel(float x, float y)
    {
        (void)x;
        (void)y;
    }

    /**
     * @brief onSignal The signal handler.
     * @param sig The signal.
     */
    virtual void onSignal(int /*sig*/);

    float32_t getCurrentFPS() const;

    EGLDisplay getEGLDisplay() const;

    void createSharedContext() const;

    WindowBase* getWindow() const;

    int getWindowWidth() const;

    int getWindowHeight() const;

    void setWindowSize(int width, int height);

    bool isOffscreen() const;

    const std::string& getArgument(const char* name) const;

protected:
    typedef std::chrono::high_resolution_clock myclock_t;
    typedef std::chrono::time_point<myclock_t> timepoint_t;

    dw::common::ProfilerCUDA* getProfilerCUDA();

    MouseView3D& getMouseView();

    ProgramArguments& getArgs();

    uint32_t getFrameIndex() const;

    myclock_t::duration convertFrecuencyToPeriod(int loopsPerSecond);

    static DriveWorksSample* instance();

private:

    static void globalSigHandler(int sig);

    virtual void resize(int width, int height);

    virtual void keyDown(int key, int scancode, int mods);

    virtual void keyUp(int key, int scancode, int mods);

    virtual void keyRepeat(int key, int scancode, int mods);

    virtual void charMods(unsigned int codepoint, int mods);

    virtual void mouseDown(int button, float x, float y, int mods);

    virtual void mouseUp(int button, float x, float y, int mods);

    virtual void mouseMove(float x, float y);

    virtual void mouseWheel(float x, float y);

    static void keyDownCb(int key, int scancode, int mods);

    static void keyUpCb(int key, int scancode, int mods);

    static void keyRepeatCb(int key, int scancode, int mods);

    static void charModsCb(unsigned int codepoint, int mods);

    static void mouseDownCb(int button, float x, float y, int mods);

    static void mouseUpCb(int button, float x, float y, int mods);

    static void mouseMoveCb(float x, float y);

    static void mouseWheelCb(float x, float y);

    static void resizeCb(int width, int height);

    static char readCommandLineChar();

    void readCLIKeyPressLoop();

private:
    dw::common::ProfilerCUDA m_profiler;
    ProgramArguments m_args;

    volatile bool m_run;
    volatile bool m_pause;
    volatile bool m_playSingleFrame;
    volatile bool m_reset;
    volatile bool m_doProfile;

    // ------------------------------------------------
    // Graphics and UI interface
    // ------------------------------------------------
    std::unique_ptr<WindowBase> m_window;
    MouseView3D m_mouseView;

    std::string m_title;
    int m_width;
    int m_height;

    // ------------------------------------------------
    // Time
    // ------------------------------------------------
    /// Defines the minimum time between calls to onProcess()
    myclock_t::duration m_processPeriod;
    /// Defines the minimum time between calls to onRender()
    myclock_t::duration m_renderPeriod;

    uint32_t m_frameIdx;
    uint32_t m_stopFrameIdx;

    /// slow down execution of the code to limit to the given process rate
    void tryToSleep(timepoint_t lastRunTime);

    // ------------------------------------------------
    // FPS calculation
    // ------------------------------------------------
    // Ring buffer to store fps samples
    static const uint32_t FPS_BUFFER_SIZE = 10;
    float32_t m_fpsBuffer[FPS_BUFFER_SIZE]{};
    uint32_t m_fpsSampleIdx = 0;
    float32_t m_currentFPS  = 0.0f;

    std::thread m_commandLineInputThread;
    bool m_commandLineInputActive;

    // ------------------------------------------------
    // singleton
    // ------------------------------------------------
    static DriveWorksSample* g_instance;
};
}
}

#endif // DRIVEWORKSAPP_HPP_
