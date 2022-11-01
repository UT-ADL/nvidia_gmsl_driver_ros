# Copyright (c) 2018-2020 NVIDIA CORPORATION.  All rights reserved.

# Interface target where all necessary libraries will be added
add_library(cublas INTERFACE)

# Remove patch number from cuda version
string(REPLACE "." ";" MAJOR_MINOR_PATCH ${CMAKE_CUDA_COMPILER_VERSION})
list(GET MAJOR_MINOR_PATCH 0 1 MAJOR_MINOR)
list(JOIN MAJOR_MINOR "." MAJOR_MINOR_STR)

# CMake configuration file
set(LIB_DIRS
    ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}/../lib
    ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}/../lib/stubs
    ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}/../lib64
    ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}/../lib64/stubs)
set(INCLUDE_DIRS ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
if(VIBRANTE_BUILD)
  if(VIBRANTE_V5Q)
    set(PLATFORM_NAME "qnx")
  else()
    set(PLATFORM_NAME "linux")
  endif()
  list(
    APPEND
    LIB_DIRS
    ${VIBRANTE_PDK}/lib-target
    ${VIBRANTE_PDK}/targetfs/usr/lib/aarch64-${PLATFORM_NAME}-gnu/stubs
    /usr/lib/aarch64-${PLATFORM_NAME}-gnu
    /usr/lib/aarch64-${PLATFORM_NAME}-gnu/stubs
    /usr/local/cuda-targets/aarch64-${PLATFORM_NAME}/${MAJOR_MINOR_STR}/lib/stubs
  )
  list(
    APPEND INCLUDE_DIRS
    /usr/local/cuda-targets/aarch64-${PLATFORM_NAME}/${MAJOR_MINOR_STR}/include
    /usr/include/aarch64-${PLATFORM_NAME}-gnu)
else()
  list(APPEND LIB_DIRS /usr/lib/nvidia-${nvidia-driver-version}
       ${DEFAULT_SYSTEM_LIB_DIR} /usr/lib/x86_64-linux-gnu
       /usr/lib/x86_64-linux-gnu/stubs)
  list(APPEND INCLUDE_DIRS /usr/include)
endif()

# Find libs
find_library(
  CUDA_cublas_LIBRARY
  NAMES cublas
  HINTS ${LIB_DIRS})
if(NOT CUDA_cublas_LIBRARY)
  message(WARNING "Could not find cublas library. Looked in ${LIB_DIRS}")
  set(cublas_FOUND False)
  return()
endif()
target_link_libraries(cublas INTERFACE ${CUDA_cublas_LIBRARY})

find_path(
  CUBLAS_INCLUDE_DIR
  NAMES cublas_v2.h
  HINTS ${INCLUDE_DIRS}
  NO_DEFAULT_PATH NO_CMAKE_PATH NO_CMAKE_SYSTEM_PATH NO_CMAKE_FIND_ROOT_PATH)
if(NOT CUBLAS_INCLUDE_DIR)
  message(WARNING "Could not find cublas header. Looked in ${INCLUDE_DIRS}")
  set(cublas_FOUND False)
  return()
endif()
target_include_directories(cublas SYSTEM INTERFACE ${CUBLAS_INCLUDE_DIR})

# cublasLt is only used in vibrante
if(VIBRANTE_BUILD)
  find_library(
    CUDA_cublasLt_LIBRARY
    NAMES cublasLt
    HINTS ${LIB_DIRS})
  if(NOT CUDA_cublasLt_LIBRARY)
    message(WARNING "Could not find cublasLt library. Looked in ${LIB_DIRS}")
    set(cublas_FOUND False)
    return()
  endif()
  target_link_libraries(cublas INTERFACE ${CUDA_cublasLt_LIBRARY})
endif()
