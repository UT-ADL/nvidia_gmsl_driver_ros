# Copyright (c) 2017-2020 NVIDIA CORPORATION.  All rights reserved.

project(glfw-src C CXX)

# Flag indicating primary and secondary backend target selection for GLFW compilation
# (might depend, e.g., on VIBRANTE-PDK version)

set(PROJ_DIR "3.2-screen")
set(PROJ_SOURCE_DIR "${PROJ_DIR}/src")

# force shared libraries for glfw
set(BUILD_SHARED_LIBS ON)
if (BUILD_SHARED_LIBS)
    set(_GLFW_BUILD_DLL 1)
endif()

# Include the base CMake file
add_subdirectory(${PROJ_SOURCE_DIR})

# Function to generate the target per window system
function(glfw_build_for_winsys TARGETNAME WINSYS DEPLIBS)
    set(INCLUDE_DIRS "")
    set(LIBRARIES ${CMAKE_DL_LIBS} ${CMAKE_THREAD_LIBS_INIT} m)

    # Window System Dependencies
    if (WINSYS MATCHES "SCREEN")
        set(_GLFW_SCREEN 1)
    elseif (WINSYS MATCHES "X11")
        set(_GLFW_X11 1)

        find_package(X11 REQUIRED)
        list(APPEND INCLUDE_DIRS ${X11_INCLUDE_DIR})
        list(APPEND LIBRARIES ${X11_LIBRARIES})

        if(X11_xf86vmode_FOUND)
            set(_GLFW_HAS_XF86VM TRUE)
            list(APPEND LIBRARIES ${X11_Xxf86vm_LIB})
        endif()

        if(X11_Xinerama_FOUND)
            list(APPEND LIBRARIES ${X11_Xinerama_LIB})
        endif()

        if(X11_Xcursor_FOUND)
            list(APPEND LIBRARIES ${X11_Xcursor_LIB})
        endif()

        if (X11_Xrandr_FOUND)
            list(APPEND LIBRARIES ${X11_Xrandr_LIB})
        endif()
    endif()

    # Configure file
    string(TOLOWER ${WINSYS} WINSYS)
    configure_file(${PROJ_SOURCE_DIR}/glfw_config.h.in ${PROJ_SOURCE_DIR}/${WINSYS}/glfw_config.h @ONLY)
    list(APPEND INCLUDE_DIRS "${CMAKE_CURRENT_BINARY_DIR}/${PROJ_SOURCE_DIR}/${WINSYS}")

    # Build Target
    glfw_add_library("glfw-${WINSYS}"
                     "${CMAKE_CURRENT_SOURCE_DIR}/${PROJ_SOURCE_DIR}"
                     "${INCLUDE_DIRS}")

    # Return library dependencies
    set(DEPLIBS ${LIBRARIES} PARENT_SCOPE)
endfunction()

# Common dependencies
set(COMMON_LIBS ${CMAKE_DL_LIBS})


# NOTE: we currently link window system dependencies at this top level because
# older CMake versions don't allow object libraries to have link dependencies.
# This can be moved back to it's original location in the src/ CMake file , but
# it requires more recent cmake versions (cmake >3.12) -> enable once
# host system cmake supports this
if(VIBRANTE_V5Q)
    message(STATUS "Building GLFW for Screen")
    glfw_build_for_winsys("glfw-screen" "SCREEN" DEPLIBS)

    add_library(${PROJECT_NAME} $<TARGET_OBJECTS:glfw-screen>)
    target_link_libraries(${PROJECT_NAME} ${DEPLIBS})

elseif(VIBRANTE_V5L AND NOT ${CMAKE_PROJECT_NAME} STREQUAL DriveworksSDK-Samples)
    message(STATUS "Building GLFW for X11 (shared)")
    glfw_build_for_winsys("glfw-x11" "X11" DEPLIBS)

    add_library(${PROJECT_NAME} $<TARGET_OBJECTS:glfw-x11>)
    target_link_libraries(${PROJECT_NAME} ${DEPLIBS})

    # For vibrante builds we are looking for vibrante libs as well
    target_link_libraries(${PROJECT_NAME} vibrante_Xlibs)

else()
    message(STATUS "Building GLFW for X11 (static)")
    glfw_build_for_winsys("glfw-x11" "X11" DEPLIBS)

    add_library(${PROJECT_NAME} $<TARGET_OBJECTS:glfw-x11>)
    target_link_libraries(${PROJECT_NAME} ${DEPLIBS})
endif()

# Add include path
set_target_properties(${PROJECT_NAME} PROPERTIES
INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/${PROJ_DIR}/include")

# we want final library to be called just libglfw.so
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "glfw")

