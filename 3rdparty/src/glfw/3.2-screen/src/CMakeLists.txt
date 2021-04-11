function(glfw_add_library TARGETNAME GLFW_SOURCE_DIR DEP_INCLUDES)

    set(common_HEADERS internal.h
                   "../include/GLFW/glfw3.h"
                   "../include/GLFW/glfw3native.h")
    set(common_SOURCES context.c init.c input.c monitor.c vulkan.c window.c)

    list(APPEND glfw_INCLUDE_DIRS ${DEP_INCLUDES})
    if (_GLFW_COCOA)
        set(glfw_HEADERS ${common_HEADERS} cocoa_platform.h cocoa_joystick.h
                         posix_tls.h nsgl_context.h)
        set(glfw_SOURCES ${common_SOURCES} cocoa_init.m cocoa_joystick.m
                         cocoa_monitor.m cocoa_window.m cocoa_time.c posix_tls.c
                         nsgl_context.m)
    elseif (_GLFW_WIN32)
        set(glfw_HEADERS ${common_HEADERS} win32_platform.h win32_joystick.h
                         wgl_context.h egl_context.h)
        set(glfw_SOURCES ${common_SOURCES} win32_init.c win32_joystick.c
                         win32_monitor.c win32_time.c win32_tls.c win32_window.c
                         wgl_context.c egl_context.c)
    elseif (_GLFW_X11)
        set(glfw_HEADERS ${common_HEADERS} x11_platform.h xkb_unicode.h
                         linux_joystick.h posix_time.h posix_tls.h glx_context.h
                         egl_context.h)
        set(glfw_SOURCES ${common_SOURCES} x11_init.c x11_monitor.c x11_window.c
                         xkb_unicode.c linux_joystick.c posix_time.c posix_tls.c
                         glx_context.c egl_context.c)
    elseif (_GLFW_WAYLAND)
        set(glfw_HEADERS ${common_HEADERS} wl_platform.h linux_joystick.h
                         posix_time.h posix_tls.h xkb_unicode.h egl_context.h)
        set(glfw_SOURCES ${common_SOURCES} wl_init.c wl_monitor.c wl_window.c
                         linux_joystick.c posix_time.c posix_tls.c xkb_unicode.c
                         egl_context.c)

        ecm_add_wayland_client_protocol(glfw_SOURCES
            PROTOCOL
            ${WAYLAND_PROTOCOLS_PKGDATADIR}/unstable/relative-pointer/relative-pointer-unstable-v1.xml
            BASENAME relative-pointer-unstable-v1)
        ecm_add_wayland_client_protocol(glfw_SOURCES
            PROTOCOL
            ${WAYLAND_PROTOCOLS_PKGDATADIR}/unstable/pointer-constraints/pointer-constraints-unstable-v1.xml
            BASENAME pointer-constraints-unstable-v1)
        list(APPEND glfw_INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR})
    elseif (_GLFW_SCREEN)
        set(glfw_HEADERS ${common_HEADERS} screen_platform.h linux_joystick.h
                         posix_time.h posix_tls.h xkb_unicode.h egl_context.h)
        set(glfw_SOURCES ${common_SOURCES} screen_init.c screen_monitor.c screen_window.c
                         linux_joystick.c posix_time.c posix_tls.c xkb_unicode.c
                         egl_context.c)
    elseif (_GLFW_MIR)
        set(glfw_HEADERS ${common_HEADERS} mir_platform.h linux_joystick.h
                         posix_time.h posix_tls.h xkb_unicode.h egl_context.h)
        set(glfw_SOURCES ${common_SOURCES} mir_init.c mir_monitor.c mir_window.c
                         linux_joystick.c posix_time.c posix_tls.c xkb_unicode.c
                         egl_context.c)
    endif()

    # Prefix relative sources and headers w/ source directory
    set(_HEADERS "")
    foreach(f ${glfw_HEADERS})
        if(NOT IS_ABSOLUTE ${f})
            set(f "${GLFW_SOURCE_DIR}/${f}")
        endif()
        list(APPEND _HEADERS ${f})
    endforeach()

    set(_SOURCES "")
    foreach(f ${glfw_SOURCES})
        if(NOT IS_ABSOLUTE ${f})
            set(f "${GLFW_SOURCE_DIR}/${f}")
        endif()
        list(APPEND _SOURCES ${f})
    endforeach()

    set(glfw_HEADERS ${_HEADERS})
    set(glfw_SOURCES ${_SOURCES})

    if (APPLE)
        # For some reason, CMake doesn't know about .m
        set_source_files_properties(${glfw_SOURCES} PROPERTIES LANGUAGE C)
    endif()

    add_library(${TARGETNAME} OBJECT ${glfw_SOURCES} ${glfw_HEADERS})
    set_target_properties(${TARGETNAME} PROPERTIES
                          VERSION ${GLFW_VERSION}
                          SOVERSION ${GLFW_VERSION_MAJOR}
                          POSITION_INDEPENDENT_CODE ON
                          INTERFACE_INCLUDE_DIRECTORIES "${GLFW_SOURCE_DIR}/../include"
                          FOLDER "GLFW3")

    target_compile_definitions(${TARGETNAME} PRIVATE -D_GLFW_USE_CONFIG_H)
    target_include_directories(${TARGETNAME} PUBLIC ${GLFW_SOURCE_DIR}/../include)
    target_include_directories(${TARGETNAME} PRIVATE ${glfw_INCLUDE_DIRS})

    # HACK: When building on MinGW, WINVER and UNICODE need to be defined before
    # the inclusion of stddef.h (by glfw3.h), which is itself included before
    # win32_platform.h.  We define them here until a saner solution can be found
    # NOTE: MinGW-w64 and Visual C++ do /not/ need this hack.
    target_compile_definitions(${TARGETNAME} PRIVATE
                               "$<$<BOOL:${MINGW}>:UNICODE;WINVER=0x0501>")

    # Enable a reasonable set of warnings (no, -Wextra is not reasonable)
    target_compile_options(${TARGETNAME} PRIVATE
                           "$<$<C_COMPILER_ID:Clang>:-Wall>"
                           "$<$<C_COMPILER_ID:GNU>:-Wall>"
                           "$<$<C_COMPILER_ID:GNU>:-Wp,-w>")

    if (BUILD_SHARED_LIBS)
        if (WIN32)
            if (MINGW)
                # Remove the lib prefix on the DLL (but not the import library
                set_target_properties(${TARGETNAME} PROPERTIES PREFIX "")

                # Add a suffix to the import library to avoid naming conflicts
                set_target_properties(${TARGETNAME} PROPERTIES IMPORT_SUFFIX "dll.a")
            else()
                # Add a suffix to the import library to avoid naming conflicts
                set_target_properties(${TARGETNAME} PROPERTIES IMPORT_SUFFIX "dll.lib")
            endif()
        elseif (APPLE)
            # Add -fno-common to work around a bug in Apple's GCC
            target_compile_options(${TARGETNAME} PRIVATE "-fno-common")

            set_target_properties(${TARGETNAME} PROPERTIES
                                  INSTALL_NAME_DIR "lib${LIB_SUFFIX}")
        elseif (UNIX)
            # Hide symbols not explicitly tagged for export from the shared library
            target_compile_options(${TARGETNAME} PRIVATE "-fvisibility=hidden")
        endif()

        target_compile_definitions(${TARGETNAME} INTERFACE -DGLFW_DLL)
    endif()

    if (MSVC)
        target_compile_definitions(${TARGETNAME} PRIVATE _CRT_SECURE_NO_WARNINGS)
    endif()

    if (GLFW_INSTALL)
        install(TARGETS ${TARGETNAME} EXPORT glfwTargets DESTINATION lib${LIB_SUFFIX})
    endif()
endfunction()

