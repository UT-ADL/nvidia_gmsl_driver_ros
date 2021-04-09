# Copyright(c)2018, NVIDIA CORPORATION.All rights reserved.

# - Try to find GLES
# Once done this will define
#  GLES_FOUND - System has GLES

find_package(PkgConfig)
pkg_check_modules(PC_GLES QUIET gles)

find_path(GLES_INCLUDE_DIR GLES3/gl31.h
          HINTS ${PC_GLES_INCLUDEDIR} ${PC_GLES_INCLUDE_DIRS} ${VIBRANTE_PDK}/include
          PATHS /usr/include)
find_path(GLES_INCLUDE_DIR2 GLES2/gl2ext.h
          HINTS ${PC_GLES_INCLUDEDIR} ${PC_GLES_INCLUDE_DIRS} ${VIBRANTE_PDK}/include
          PATHS /usr/include)

find_library(GLES_LIBRARY GLESv2
             HINTS ${PC_GLES_LIBDIR} ${PC_GLES_LIBRARY_DIRS} ${VIBRANTE_PDK}/lib-target
             PATHS /usr/lib)

if ((NOT GLES_INCLUDE_DIR) OR (NOT GLES_INCLUDE_DIR2))
    if (LINUX)
        message(FATAL_ERROR "GLES header not found. Use: 'sudo apt install libgles2-mesa-dev'")
    endif()
    message(FATAL_ERROR "GLES header not found")
endif()

if (GLES_LIBRARY)
    add_library(gles SHARED IMPORTED)
    set_target_properties(gles PROPERTIES
                          IMPORTED_LOCATION ${GLES_LIBRARY}
                          INTERFACE_SYSTEM_INCLUDE_DIRECTORIES ${GLES_INCLUDE_DIR}
                          INTERFACE_LINK_LIBRARIES ${GLES_LIBRARY})
    set_property(TARGET gles APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
    set_target_properties(gles PROPERTIES
                          IMPORTED_LOCATION_RELEASE ${GLES_LIBRARY})
    set_target_properties(gles PROPERTIES MAP_IMPORTED_CONFIG_PROFILE "Release")
    set(GLES_FOUND TRUE)
endif()
