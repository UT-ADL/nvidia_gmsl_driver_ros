#include "internal.h"

static GLFWbool setWindowProperty(_GLFWwindow* window, _GLFWfbconfig *fbconfig,
                                  _GLFWwndconfig *wndconfig, _GLFWctxconfig *ctxconfig)
{
    int format = SCREEN_FORMAT_RGBA8888;
    int usage = SCREEN_USAGE_OPENGL_ES2;
    int transparency = SCREEN_TRANSPARENCY_NONE;
    EGLint interval = 1;

    // Set the requested display
    if(_glfw.screen.screen_set_window_property_pv(window->screen.screen_win,
                                                  SCREEN_PROPERTY_DISPLAY,
                                                  (void **) &_glfw.screen.screen_display))
    {
        _glfwInputError(GLFW_PLATFORM_ERROR,
                        "Screen: Failed to set display");
        return GLFW_FALSE;
    }

    if (fbconfig->redBits == 8 &&
        fbconfig->greenBits == 8 &&
        fbconfig->blueBits == 8)
    {
        if (fbconfig->alphaBits == 8)
            format = SCREEN_FORMAT_RGBA8888;
        else
            format = SCREEN_FORMAT_RGBX8888;
    } else if (fbconfig->redBits == 5 &&
               fbconfig->greenBits == 6 &&
               fbconfig->blueBits == 5)
        format = SCREEN_FORMAT_RGB565;
    else
        return GLFW_FALSE;

    // Set window property SCREEN_PROPERTY_FORMAT
    if(_glfw.screen.screen_set_window_property_iv(window->screen.screen_win,
                                                  SCREEN_PROPERTY_FORMAT,
                                                  &format))
    {
        _glfwInputError(GLFW_PLATFORM_ERROR,
                        "Screen: Failed to set window property SCREEN_PROPERTY_FORMAT");
        return GLFW_FALSE;
    }

    // Set window property SCREEN_PROPERTY_USAGE
    if(_glfw.screen.screen_set_window_property_iv(window->screen.screen_win,
                                                  SCREEN_PROPERTY_USAGE,
                                                  &usage))
    {
        _glfwInputError(GLFW_PLATFORM_ERROR,
                        "Screen: Failed to set window property "
                        "SCREEN_PROPERTY_USAGE");
        return GLFW_FALSE;
    }

    // Set window property SCREEN_PROPERTY_SWAP_INTERVAL
    if(_glfw.screen.screen_set_window_property_iv(window->screen.screen_win,
                                                  SCREEN_PROPERTY_SWAP_INTERVAL,
                                                  &interval))
    {
        _glfwInputError(GLFW_PLATFORM_ERROR,
                        "Screen: Failed to set window property SCREEN_PROPERTY_SWAP_INTERVAL");
        return GLFW_FALSE;
    }

    if(_glfw.screen.screen_set_window_property_iv(window->screen.screen_win,
                                                  SCREEN_PROPERTY_TRANSPARENCY,
                                                  &transparency))
    {
        _glfwInputError(GLFW_PLATFORM_ERROR,
                        "Screen: Failed to set window property"
                        " SCREEN_TRANSPARENCY_NONE");
        return GLFW_FALSE;
    }

    {
       int windowSize[2] = {wndconfig->width, wndconfig->height};
       if(_glfw.screen.screen_set_window_property_iv(window->screen.screen_win,
                                                     SCREEN_PROPERTY_SIZE,
                                                     windowSize))
       {
           _glfwInputError(GLFW_PLATFORM_ERROR,
                           "Screen: Failed to set window property SCREEN_PROPERTY_SIZE");
           return GLFW_FALSE;
       }
    }

    return GLFW_TRUE;
}
///////////////////////////////////////////////////////////////////////////
////////            GLFW platfor API                          /////////////
///////////////////////////////////////////////////////////////////////////

int _glfwPlatformCreateWindow(_GLFWwindow* window,
                              const _GLFWwndconfig* wndconfig,
                              const _GLFWctxconfig* ctxconfig,
                              const _GLFWfbconfig* fbconfig)
{
    int nbuffers = 2;

    if (!_glfw.screen.screen_ctx)
        return GLFW_FALSE;

    // Create a native window
    if(_glfw.screen.screen_create_window(&window->screen.screen_win,
                                         _glfw.screen.screen_ctx))

    {
        _glfwInputError(GLFW_PLATFORM_ERROR,
                        "Screen: Failed to create native window");
        return GLFW_FALSE;
    }


    if (!setWindowProperty(window, fbconfig, wndconfig, ctxconfig))
        return GLFW_FALSE;

    // Create QNX CAR 2.1 window buffers
    if(_glfw.screen.screen_create_window_buffers(window->screen.screen_win, nbuffers))
    {
        _glfwInputError(GLFW_PLATFORM_ERROR,
                        "Screen: Failed to create window buffers");
        return GLFW_FALSE;
    }

    // Create context
    if (!_glfwCreateContextEGL(window, ctxconfig, fbconfig))
        return GLFW_FALSE;

    return GLFW_TRUE;
}

void _glfwPlatformDestroyWindow(_GLFWwindow* window)
{
    if (window->context.client != GLFW_NO_API)
        window->context.destroy(window);

    if (window->screen.screen_win)
        _glfw.screen.screen_destroy_window(window->screen.screen_win);

    if(_glfw.screen.screen_ev)
        _glfw.screen.screen_destroy_event(_glfw.screen.screen_ev);
}

void _glfwPlatformSetWindowTitle(_GLFWwindow* window, const char* title)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformSetWindowTitle is not supported");
}

void _glfwPlatformSetWindowIcon(_GLFWwindow* window,
                                int count, const GLFWimage* images)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformSetWindowIcon is not supported");
}

void _glfwPlatformGetWindowPos(_GLFWwindow* window, int* xpos, int* ypos)
{
    int pos[2] = {0, 0};
    if(_glfw.screen.screen_get_window_property_iv(window->screen.screen_win,
                                                  SCREEN_PROPERTY_POSITION,
                                                  &pos))
    {
        _glfwInputError(GLFW_PLATFORM_ERROR,
                        "Screen: Unable to get Window Position");
        return;
    }
    if (xpos)
        *xpos = pos[0];
    if (ypos)
        *ypos = pos[1];
}

void _glfwPlatformSetWindowPos(_GLFWwindow* window, int xpos, int ypos)
{
    int pos[2] = {xpos, ypos};
    if (_glfw.screen.screen_set_window_property_iv(window->screen.screen_win,
                                                   SCREEN_PROPERTY_POSITION,
                                                   pos))
    {
         _glfwInputError(GLFW_PLATFORM_ERROR,
                        "Screen: Unable to set Window Position");
        return;
    }
}

void _glfwPlatformGetWindowSize(_GLFWwindow* window, int* width, int* height)
{
    int windowSize[2];
    if(_glfw.screen.screen_get_window_property_iv(window->screen.screen_win,
                                                  SCREEN_PROPERTY_SIZE,
                                                  windowSize))
    {
        _glfwInputError(GLFW_PLATFORM_ERROR,
                        "Screen: Failed to get window size");
        return;
    }
    if (width)
        *width = windowSize[0];
    if (height)
        *height = windowSize[1];
}

void _glfwPlatformSetWindowSize(_GLFWwindow* window, int width, int height)
{
    int windowSize[2] = {width, height};
    if(_glfw.screen.screen_set_window_property_iv(window->screen.screen_win,
                                                  SCREEN_PROPERTY_SIZE,
                                                  windowSize))
    {
        _glfwInputError(GLFW_PLATFORM_ERROR,
                        "Screen: Failed to set window size");
        return;
    }
}

void _glfwPlatformSetWindowSizeLimits(_GLFWwindow* window,
                                      int minwidth, int minheight,
                                      int maxwidth, int maxheight)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformSetWindowSizeLimits not implemented");
}

void _glfwPlatformSetWindowAspectRatio(_GLFWwindow* window, int numer, int denom)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformSetWindowAspectRatio not implemented");
}

void _glfwPlatformGetFramebufferSize(_GLFWwindow* window, int* width, int* height)
{
    _glfwPlatformGetWindowSize(window, width, height);
}

void _glfwPlatformGetWindowFrameSize(_GLFWwindow* window,
                                     int* left, int* top,
                                     int* right, int* bottom)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformGetWindowFrameSize not implemented");
}

void _glfwPlatformIconifyWindow(_GLFWwindow* window)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformIconifyWindow not implemented");
}

void _glfwPlatformRestoreWindow(_GLFWwindow* window)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformRestoreWindow not implemented");
}

void _glfwPlatformMaximizeWindow(_GLFWwindow* window)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformMaximizeWindow not implemented");
}

void _glfwPlatformShowWindow(_GLFWwindow* window)
{
    return;
}

void _glfwPlatformUnhideWindow(_GLFWwindow* window)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformUnhideWindow not implemented");
}

void _glfwPlatformFocusWindow(_GLFWwindow* window)
{
    return;
}

void _glfwPlatformSetWindowMonitor(_GLFWwindow* window,
                                   _GLFWmonitor* monitor,
                                   int xpos, int ypos,
                                   int width, int height,
                                   int refreshRate)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformSetWindowMonitor not implemented");
}

void _glfwPlatformHideWindow(_GLFWwindow* window)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformHideWindow not implemented");
}

int _glfwPlatformWindowFocused(_GLFWwindow* window)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformWindowFocused not implemented");
    return GLFW_FALSE;
}

int _glfwPlatformWindowIconified(_GLFWwindow* window)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformWindowIconified not implemented");
    return GLFW_FALSE;
}

int _glfwPlatformWindowVisible(_GLFWwindow* window)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformWindowVisible not implemented");
    return GLFW_FALSE;
}

int _glfwPlatformWindowMaximized(_GLFWwindow* window)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformWindowMaximized not implemented");
    return 0;
}

void _glfwPlatformPollEvents(void)
{
    return;
}

void _glfwPlatformWaitEvents(void)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformWaitEvents not supported");
}

void _glfwPlatformWaitEventsTimeout(double timeout)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformWaitEventsTimeout not supported");
}

void _glfwPlatformPostEmptyEvent(void)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformPostEmptyEvent not supported");
}

void _glfwPlatformGetCursorPos(_GLFWwindow* window, double* xpos, double* ypos)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformGetCursorPos not supported");
}

void _glfwPlatformSetCursorPos(_GLFWwindow* window, double x, double y)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformSetCursorPos not supported");
}

void _glfwPlatformSetCursorMode(_GLFWwindow* window, int mode)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformSetCursorMode not supported");
}

const char* _glfwPlatformGetKeyName(int key, int scancode)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformGetKeyName not supported");
    return NULL;
}

int _glfwPlatformCreateCursor(_GLFWcursor* cursor,
                              const GLFWimage* image,
                              int xhot, int yhot)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformCreateCursor not supported");
    return GLFW_FALSE;
}

int _glfwPlatformCreateStandardCursor(_GLFWcursor* cursor, int shape)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformCreateStandardCursor not supported");
    return GLFW_FALSE;
}

void _glfwPlatformDestroyCursor(_GLFWcursor* cursor)
{
    return;
}

void _glfwPlatformSetCursor(_GLFWwindow* window, _GLFWcursor* cursor)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformSetCursor not supported");
}

void _glfwPlatformSetClipboardString(_GLFWwindow* window, const char* string)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformSetClipboardString not supported");
}

char** _glfwPlatformGetRequiredInstanceExtensions(uint32_t* count)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformGetRequiredInstanceExtensions not supported");
    return NULL;
}

int _glfwPlatformGetPhysicalDevicePresentationSupport(VkInstance instance,
                                                      VkPhysicalDevice device,
                                                      uint32_t queuefamily)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformGetPhysicalDevicePresentationSupport not supported");
    return 0;
}

const char* _glfwPlatformGetClipboardString(_GLFWwindow* window)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformGetClipboardString not supported");
    return NULL;
}
VkResult _glfwPlatformCreateWindowSurface(VkInstance instance,
                                          _GLFWwindow* window,
                                          const VkAllocationCallbacks* allocator,
                                          VkSurfaceKHR* surface)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformCreateWindowSurface not supported");
    return (VkResult)NULL;
}

