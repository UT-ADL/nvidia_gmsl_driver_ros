#include "internal.h"

static GLFWbool initializeFunctionPtrs(void)
{
    _glfw.screen.libscreen = _glfw_dlopen("libscreen.so");
    if (!_glfw.screen.libscreen)
    {
        _glfw.screen.libscreen = _glfw_dlopen("libscreen.so.1");
        if (!_glfw.screen.libscreen)
            return GLFW_FALSE;
    }

    _glfw.screen.screen_create_context = (PFNSCREENCREATECONTEXT)
        _glfw_dlsym(_glfw.screen.libscreen, "screen_create_context");
    _glfw.screen.screen_destroy_context = (PFNSCREENDESTROYCONTEXT)
        _glfw_dlsym(_glfw.screen.libscreen, "screen_destroy_context");
    _glfw.screen.screen_get_context_property_iv = (PFNSCREENGETCONTEXTPROPERTYIV)
        _glfw_dlsym(_glfw.screen.libscreen, "screen_get_context_property_iv");
    _glfw.screen.screen_get_context_property_pv = (PFNSCREENGETCONTEXTPROPERTYPV)
        _glfw_dlsym(_glfw.screen.libscreen, "screen_get_context_property_pv");
    _glfw.screen.screen_create_window = (PFNSCREENCREATEWINDOW)
        _glfw_dlsym(_glfw.screen.libscreen, "screen_create_window");
    _glfw.screen.screen_set_window_property_pv = (PFNSCREENSETWINDOWPROPERTYPV)
        _glfw_dlsym(_glfw.screen.libscreen, "screen_set_window_property_pv");
    _glfw.screen.screen_set_window_property_iv = (PFNSCREENSETWINDOWPROPERTYIV)
        _glfw_dlsym(_glfw.screen.libscreen, "screen_set_window_property_iv");
    _glfw.screen.screen_get_window_property_iv = (PFNSCREENGETWINDOWPROPERTYIV)
        _glfw_dlsym(_glfw.screen.libscreen, "screen_get_window_property_iv");
    _glfw.screen.screen_create_window_buffers = (PFNXSCREENCREATEWINDOWBUFFERS)
        _glfw_dlsym(_glfw.screen.libscreen, "screen_create_window_buffers");
    _glfw.screen.screen_get_event = (PFNSCREENGETEVENT)
        _glfw_dlsym(_glfw.screen.libscreen, "screen_get_event");
    _glfw.screen.screen_get_event_property_iv = (PFNSCREENGETEVENTPROPERTYIV)
        _glfw_dlsym(_glfw.screen.libscreen, "screen_get_event_property_iv");
    _glfw.screen.screen_create_event = (PFNSCREENCREATEEVENT)
        _glfw_dlsym(_glfw.screen.libscreen, "screen_create_event");
    _glfw.screen.screen_destroy_window = (PFNSCREENDESTROYWINDOW)
        _glfw_dlsym(_glfw.screen.libscreen, "screen_destroy_window");
    _glfw.screen.screen_destroy_event = (PFNSCREENDESTROYEVENT)
        _glfw_dlsym(_glfw.screen.libscreen, "screen_destroy_event");
    _glfw.screen.screen_create_pixmap = (PFNSCREENCREATEPIXMAP)
        _glfw_dlsym(_glfw.screen.libscreen, "screen_create_pixmap");
    _glfw.screen.screen_set_pixmap_property_iv = (PFNSCREENSETPIXMAPPROPERTYIV)
        _glfw_dlsym(_glfw.screen.libscreen, "screen_set_pixmap_property_iv");
    _glfw.screen.screen_destroy_pixmap = (PFNSCREENDESTROYPIXMAP)
        _glfw_dlsym(_glfw.screen.libscreen, "screen_destroy_pixmap");
    _glfw.screen.screen_get_display_property_iv = (PFNSCREENGETDISPLAYPROPERTYIV)
        _glfw_dlsym(_glfw.screen.libscreen, "screen_get_display_property_iv");
    _glfw.screen.screen_flush_context = (PFNSCREENFLUSHCONTEXT)
        _glfw_dlsym(_glfw.screen.libscreen, "screen_flush_context");
    _glfw.screen.screen_get_display_property_pv = (PFNSCREENGETDISPLAYPROPERTYPV)
        _glfw_dlsym(_glfw.screen.libscreen, "screen_get_display_property_pv");
    _glfw.screen.screen_get_display_modes = (PFNSCREENGETDISPLAYMODES)
        _glfw_dlsym(_glfw.screen.libscreen, "screen_get_display_modes");

    return GLFW_TRUE;
}

//////////////////////////////////////////////////////////////////////////
//////                       GLFW platform API                      //////
//////////////////////////////////////////////////////////////////////////

int _glfwPlatformInit(void)
{
    int rc;

    if (!initializeFunctionPtrs())
        return GLFW_FALSE;

    _glfw.screen.screen_ctx = 0;

    // Create Screen Context
    rc = _glfw.screen.screen_create_context(&_glfw.screen.screen_ctx, 0);
    if (rc)
    {
        _glfwInputError(GLFW_PLATFORM_ERROR,
                        "Screen: Unable to create context");
        return GLFW_FALSE;
    }

    if (!_glfwInitThreadLocalStoragePOSIX())
        return GLFW_FALSE;

    if (!_glfwInitEGL())
        return GLFW_FALSE;

    _glfwInitTimerPOSIX();

    return GLFW_TRUE;
}

void _glfwPlatformTerminate(void)
{
    _glfwTerminateEGL();
    _glfwTerminateThreadLocalStoragePOSIX();

    if (_glfw.screen.screen_ctx)
        _glfw.screen.screen_destroy_context(_glfw.screen.screen_ctx);

    if (_glfw.screen.libscreen)
        _glfw_dlclose(_glfw.screen.libscreen);
}

const char* _glfwPlatformGetVersionString(void)
{
    return _GLFW_VERSION_NUMBER "Screen"
#if defined(_GLFW_EGL)
        " EGL"
#endif
#if defined(_GLFW_BUILD_DLL)
        " shared"
#endif
        ;
}
