# Copyright (c) 2016-2020 NVIDIA CORPORATION.  All rights reserved.
# Edited by Maxandre Ogeret - University of Tartu

cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

#-------------------------------------------------------------------------------
# Set basic configurations
#-------------------------------------------------------------------------------
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(SamplesSetBuildType) # Set the build type before project is created

set(SDK_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(SDK_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})

#-------------------------------------------------------------------------------
# CUDA host compiler must be set before CUDA is enabled as a language
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
# Set CUDA_DIR
#-------------------------------------------------------------------------------
if (DEFINED CUDA_DIR)
    if((DEFINED CUDA_TOOLKIT_ROOT_DIR) AND (NOT CUDA_TOOLKIT_ROOT_DIR STREQUAL CUDA_DIR))
        message(FATAL_ERROR "Cannot set both CUDA_DIR and (legacy) CUDA_TOOLKIT_ROOT_DIR")
    endif()
elseif (DEFINED CUDA_TOOLKIT_ROOT_DIR)
    message(WARNING "Please set CUDA_DIR instead of (legacy) CUDA_TOOLKIT_ROOT_DIR")
    set(CUDA_DIR  ${CUDA_TOOLKIT_ROOT_DIR} CACHE PATH "CUDA Toolkit location.")
else()
    set(CUDA_DIR  "/usr/local/cuda/" CACHE PATH "CUDA Toolkit location.")
endif()
if(NOT CMAKE_CUDA_COMPILER)
    set(CMAKE_CUDA_COMPILER "${CUDA_DIR}/bin/nvcc")
endif()
set(CMAKE_CUDA_HOST_COMPILER ${CMAKE_CXX_COMPILER})
enable_language(CUDA)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CUDA_STANDARD 11)
set(CMAKE_C_STANDARD 99)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

#-------------------------------------------------------------------------------
# System packages
#-------------------------------------------------------------------------------
find_package(EGL)
find_package(Threads REQUIRED)

#-------------------------------------------------------------------------------
# Basic configuration
#-------------------------------------------------------------------------------
include(ArchConfiguration)
include(ResourcesConfiguration)
include(Samples3rdparty)

#-------------------------------------------------------------------------------
# Driveworks SDK
#-------------------------------------------------------------------------------
find_package(cuda-libs REQUIRED CONFIG HINTS ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/shared/cuda-libs)
find_package(cublas REQUIRED CONFIG HINTS ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/shared/cublas)
find_package(driveworks REQUIRED CONFIG HINTS /usr/local/driveworks/cmake)
find_package(driveworks_visualization REQUIRED CONFIG HINTS /usr/local/driveworks/cmake)
find_package(dwdynamicmemory REQUIRED CONFIG HINTS /usr/local/driveworks/cmake)

set(Driveworks_LIBRARIES
        cuda-libs
        cublas
        driveworks
        driveworks_visualization)

if(VIBRANTE)
    list(APPEND Driveworks_LIBRARIES ${vibrante_LIBRARIES} ${vibrante_Xlibs_LIBRARIES} nvmedia)
endif()

if (VIBRANTE OR DW_EXPERIMENTAL_FORCE_EGL)
    list(APPEND Driveworks_LIBRARIES ${EGL_LIBRARIES})
endif()

#-------------------------------------------------------------------------------
# END CONFIG
#-------------------------------------------------------------------------------