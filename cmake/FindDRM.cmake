# Copyright(c)2018, NVIDIA CORPORATION.All rights reserved.

# - Try to find DRM
# Once done this will define
#  DRM_FOUND - System has DRM

find_package(PkgConfig)
pkg_check_modules(PC_DRM QUIET libdrm)

find_path(DRM_INCLUDE_DIR /drm.h
          /usr/include/libdrm /usr/include/drm)

find_library(DRM_LIBRARY drm
             PATHS /usr/lib)

if (NOT DRM_INCLUDE_DIR)
    if (LINUX)
        message(WARNING "DRM header not found. Use: 'sudo apt install libdrm-dev'")
    endif()
    message(WARNING "DRM header not found")
else()
    message(STATUS "Found: ${DRM_LIBRARY}")
    message(STATUS "Header at: ${DRM_INCLUDE_DIR}")
endif()

if (DRM_LIBRARY)
    add_library(drm SHARED IMPORTED)
    set_target_properties(drm PROPERTIES
                          IMPORTED_LOCATION ${DRM_LIBRARY}
                          INTERFACE_INCLUDE_DIRECTORIES ${DRM_INCLUDE_DIR}
                          INTERFACE_LINK_LIBRARIES ${DRM_LIBRARY})
    set_property(TARGET drm APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
    set_target_properties(drm PROPERTIES
                          IMPORTED_LOCATION_RELEASE ${DRM_LIBRARY})
    set_target_properties(drm PROPERTIES MAP_IMPORTED_CONFIG_PROFILE "Release")
    set(DRM_FOUND TRUE)
else()
    add_library(drm INTERFACE)
endif()
