#include "internal.h"
#include <stdlib.h>

/////////////////////////////////////////////////////////////////////////////
///////////               GLFW platform API                    //////////////
/////////////////////////////////////////////////////////////////////////////

_GLFWmonitor** _glfwPlatformGetMonitors(int* count)
{
    int i;
    int attached = 0;
    int displayCount = 0;
    screen_display_t *displayHandle;
    _GLFWmonitor** monitors = NULL;

    *count = 0;

    if(_glfw.screen.screen_get_context_property_iv(_glfw.screen.screen_ctx,
                                                   SCREEN_PROPERTY_DISPLAY_COUNT,
                                                   &displayCount))
    {
        _glfwInputError(GLFW_PLATFORM_ERROR,
                        "Screen: Unable to get the number of displays");
        goto fail;
    }

    displayHandle = calloc(displayCount, sizeof(screen_display_t));
    if (!displayHandle)
        goto fail;

    monitors = calloc(displayCount, sizeof(_GLFWmonitor*));
    if (!monitors)
        goto fail1;

    if(_glfw.screen.screen_get_context_property_pv(_glfw.screen.screen_ctx,
                                                   SCREEN_PROPERTY_DISPLAYS,
                                                   (void **) displayHandle))
    {
        _glfwInputError(GLFW_PLATFORM_ERROR,
                        "Screen: Unable to get displays");
        goto fail1;
    }

    for (i = 0; i < displayCount; i++)
    {
        _GLFWmonitor* monitor;
        monitor = calloc(1, sizeof(_GLFWmonitor));
        if (!monitor)
            goto fail2;

        monitor->screen.display = displayHandle[i];
        if(_glfw.screen.screen_get_display_property_iv(monitor->screen.display,
                                                       SCREEN_PROPERTY_ATTACHED,
                                                       &attached))
        {
            _glfwInputError(GLFW_PLATFORM_ERROR,
                            "Screen: Unable to query the state of display");
            goto fail2;
        }
        if(attached)
            monitor->screen.state = ATTACHED;
        else
            monitor->screen.state = DETACHED;

        monitors[i] = monitor;
    }

    for(i = 0; i < displayCount; i++)
    {
        if(monitors[i]->screen.state == ATTACHED)
        {
            _glfw.screen.screen_display = monitors[i]->screen.display;
            break;
        }
    }

    if (!_glfw.screen.screen_display)
    {
        _glfwInputError(GLFW_PLATFORM_ERROR,
                        "Screen: No displays attached");
        goto fail2;
    }

    free(displayHandle);
    *count = displayCount;

    return monitors;

fail2:
    for (i = 0; i < displayCount; i++)
    {
        if (monitors[i])
            free(monitors[i]);
    }
fail1:
    if (monitors)
        free(monitors);
    if(displayHandle)
        free(displayHandle);
fail:
    return NULL;
}

GLFWbool _glfwPlatformIsSameMonitor(_GLFWmonitor* first, _GLFWmonitor* second)
{
    int id1 = 0, id2 = 0;

    if (_glfw.screen.screen_get_display_property_iv(first->screen.display,
                                                    SCREEN_PROPERTY_ID, &id1))
    {
        _glfwInputError(GLFW_PLATFORM_ERROR,
                        "Screen: Unable to get display Id for first monitor");
        return GLFW_FALSE;
    }

    if (_glfw.screen.screen_get_display_property_iv(second->screen.display,
                                                    SCREEN_PROPERTY_ID, &id2))
    {
        _glfwInputError(GLFW_PLATFORM_ERROR,
                        "Screen: Unable to get display Id for second monitor");
        return GLFW_FALSE;
    }

    if (id1 == id2)
        return GLFW_TRUE;

    return GLFW_FALSE;
}

void _glfwPlatformGetMonitorPos(_GLFWmonitor* monitor, int* xpos, int* ypos)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformGetMonitorPos not implemented");
}

GLFWvidmode* _glfwPlatformGetVideoModes(_GLFWmonitor* monitor, int* found)
{
    GLFWvidmode *modes = NULL;
    int i, modesCount = 0;
    screen_display_mode_t *screen_modes;

    if(_glfw.screen.screen_get_display_property_iv(monitor->screen.display,
                                                   SCREEN_PROPERTY_MODE_COUNT,
                                                   &modesCount))
    {
        _glfwInputError(GLFW_PLATFORM_ERROR,
                        "Screen: Unable to get the number of modes");
        goto fail;
    }

    modes = calloc(modesCount, sizeof(GLFWvidmode));
    if (!modes)
        goto fail;

    screen_modes = calloc(modesCount, sizeof(screen_display_mode_t));
    if (!screen_modes)
        goto fail1;

    if(_glfw.screen.screen_get_display_modes(monitor->screen.display,
                                             modesCount, screen_modes))
    {
        _glfwInputError(GLFW_PLATFORM_ERROR,
                        "Screen: Unable to get display modes");
        goto fail1;
    }

    for (i = 0;  i < modesCount;  i++)
    {
        modes[i].width = screen_modes[i].width;
        modes[i].height = screen_modes[i].height;
        modes[i].refreshRate = screen_modes[i].refresh;
    }

    free(screen_modes);

    *found = modesCount;
    return modes;

fail1:
    if (screen_modes)
        free(screen_modes);
    if (modes)
        free(modes);
fail:
    return NULL;
}

void _glfwPlatformGetVideoMode(_GLFWmonitor* monitor, GLFWvidmode* mode)
{
    screen_display_mode_t screen_mode;
    if(_glfw.screen.screen_get_display_property_pv(monitor->screen.display,
                                                   SCREEN_PROPERTY_MODE,
                                                   (void**)&screen_mode))
    {
        _glfwInputError(GLFW_PLATFORM_ERROR,
                        "Screen: Unable to get Video Mode");
        return;
    }

    mode->width = screen_mode.width;
    mode->height = screen_mode.height;
    mode->refreshRate = screen_mode.refresh;
}

void _glfwPlatformGetGammaRamp(_GLFWmonitor* monitor, GLFWgammaramp* ramp)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformGetGammaRamp not implemented");
}

void _glfwPlatformSetGammaRamp(_GLFWmonitor* monitor, const GLFWgammaramp* ramp)
{
    _glfwInputError(GLFW_PLATFORM_ERROR,
                    "Screen: _glfwPlatformSetGammaRamp not implemented");
}
