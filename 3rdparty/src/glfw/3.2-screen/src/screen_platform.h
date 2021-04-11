#ifndef _glfw3_screen_h_
#define _glfw3_screen_h_

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <screen/screen.h>

#include <dlfcn.h>
#include "posix_time.h"
#include "linux_joystick.h"
#include "posix_tls.h"

#include "egl_context.h"

#define _glfw_dlopen(name) dlopen(name, RTLD_LAZY | RTLD_LOCAL)
#define _glfw_dlclose(handle) dlclose(handle)
#define _glfw_dlsym(handle, name) dlsym(handle, name)

#define _GLFW_EGL_NATIVE_WINDOW ((EGLNativeWindowType)window->screen.screen_win)
#define _GLFW_EGL_NATIVE_DISPLAY EGL_DEFAULT_DISPLAY

#define _GLFW_PLATFORM_WINDOW_STATE         _GLFWwindowScreen screen
#define _GLFW_PLATFORM_LIBRARY_WINDOW_STATE _GLFWlibraryScreen screen
#define _GLFW_PLATFORM_MONITOR_STATE        _GLFWmonitorScreen screen
#define _GLFW_PLATFORM_CURSOR_STATE         _GLFWcursorScreen screen

#define _GLFW_PLATFORM_CONTEXT_STATE
#define _GLFW_PLATFORM_LIBRARY_CONTEXT_STATE

typedef int (*PFNSCREENCREATECONTEXT)(screen_context_t *, int);
typedef int (*PFNSCREENDESTROYCONTEXT)(screen_context_t);
typedef int (*PFNSCREENGETCONTEXTPROPERTYIV)(screen_context_t ctx, int pname, int *param);
typedef int (*PFNSCREENGETCONTEXTPROPERTYPV)(screen_context_t ctx, int pname, void **param);
typedef int (*PFNSCREENCREATEWINDOW)(screen_window_t *, screen_context_t);
typedef int (*PFNSCREENSETWINDOWPROPERTYIV)(screen_window_t, int, const int *);
typedef int (*PFNSCREENSETWINDOWPROPERTYPV)(screen_window_t, int, void **);
typedef int (*PFNSCREENGETWINDOWPROPERTYIV)(screen_window_t, int, int *);
typedef int (*PFNXSCREENCREATEWINDOWBUFFERS)(screen_window_t, int);
typedef int (*PFNSCREENGETEVENT)(screen_context_t, screen_event_t, uint64_t);
typedef int (*PFNSCREENGETEVENTPROPERTYIV)(screen_event_t, int, int *);
typedef int (*PFNSCREENCREATEEVENT)(screen_event_t *);
typedef int (*PFNSCREENDESTROYWINDOW)(screen_window_t);
typedef int (*PFNSCREENDESTROYEVENT)(screen_event_t);
typedef int (*PFNSCREENCREATEPIXMAP)(screen_pixmap_t *, screen_context_t);
typedef int (*PFNSCREENSETPIXMAPPROPERTYIV)(screen_pixmap_t, int, const int *);
typedef int (*PFNSCREENDESTROYPIXMAP)(screen_pixmap_t);
typedef int (*PFNSCREENGETDISPLAYPROPERTYIV)(screen_display_t disp, int pname, int *param);
typedef int (*PFNSCREENFLUSHCONTEXT)(screen_context_t ctx, int flags);
typedef int (*PFNSCREENGETDISPLAYPROPERTYPV)(screen_context_t ctx, int pname, void **param);
typedef int (*PFNSCREENGETDISPLAYMODES) (screen_display_t disp, int n, screen_display_mode_t *param);

// Screen-specific per-window data
//
typedef struct _GLFWwindowScreen
{
   screen_window_t screen_win;
} _GLFWwindowScreen;

// Screen-specific global data
//
typedef struct _GLFWlibraryScreen
{
    void *libscreen;
    screen_context_t screen_ctx;
    screen_display_t screen_display;
    screen_event_t screen_ev;

    PFNSCREENCREATECONTEXT        screen_create_context;
    PFNSCREENDESTROYCONTEXT       screen_destroy_context;
    PFNSCREENGETCONTEXTPROPERTYIV screen_get_context_property_iv;
    PFNSCREENGETCONTEXTPROPERTYPV screen_get_context_property_pv;
    PFNSCREENCREATEWINDOW         screen_create_window;
    PFNSCREENSETWINDOWPROPERTYIV  screen_set_window_property_iv;
    PFNSCREENSETWINDOWPROPERTYPV  screen_set_window_property_pv;
    PFNSCREENGETWINDOWPROPERTYIV  screen_get_window_property_iv;
    PFNXSCREENCREATEWINDOWBUFFERS screen_create_window_buffers;
    PFNSCREENGETEVENT             screen_get_event;
    PFNSCREENGETEVENTPROPERTYIV   screen_get_event_property_iv;
    PFNSCREENCREATEEVENT          screen_create_event;
    PFNSCREENDESTROYWINDOW        screen_destroy_window;
    PFNSCREENDESTROYEVENT         screen_destroy_event;
    PFNSCREENCREATEPIXMAP         screen_create_pixmap;
    PFNSCREENSETPIXMAPPROPERTYIV  screen_set_pixmap_property_iv;
    PFNSCREENDESTROYPIXMAP        screen_destroy_pixmap;
    PFNSCREENGETDISPLAYPROPERTYIV screen_get_display_property_iv;
    PFNSCREENFLUSHCONTEXT         screen_flush_context;
    PFNSCREENGETDISPLAYPROPERTYPV screen_get_display_property_pv;
    PFNSCREENGETDISPLAYMODES      screen_get_display_modes;
} _GLFWlibraryScreen;

// Screen-specific per-monitor data
//
typedef struct _GLFWmonitorScreen
{
    screen_display_t display;
    enum { DETACHED, ATTACHED } state;
} _GLFWmonitorScreen;

// Screen-specific per-cursor data
//
typedef struct _GLFWcursorScreen
{
} _GLFWcursorScreen;

#endif // _glfw3_screen_platform_h_
