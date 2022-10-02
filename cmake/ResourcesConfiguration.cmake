# Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.

# -------------------------------------------------------------------------------
# Set CuDNN and TensorRT version TRT_VERSION is cmake command line parameter
# -------------------------------------------------------------------------------

if(DEFINED TRT_VERSION)
  if(VIBRANTE_BUILD)
    if(TRT_VERSION STREQUAL "6.0.0.11" AND VIBRANTE_PDK_VERSION STREQUAL
                                           5.1.12.0)
      set(CUDNN_VERSION 7.6.3.8)
      set(DIT_VERSION_STR
          06_00_00_11
          CACHE INTERNAL "")
      set(RESOURCES_VERSION trt_06_00_00_11)
      set(NVDLA_VERSION 01_04_00)
      set(REQUIRED_CUDA_VERSION 10.2)
      message(
        STATUS "Building with TRT version ${TRT_VERSION} and cudnn version "
               "${CUDNN_VERSION}, pdk version ${VIBRANTE_PDK_VERSION}")
    elseif(TRT_VERSION STREQUAL "6.2.0.3"
           AND (VIBRANTE_PDK_VERSION STREQUAL 5.1.15.0 OR VIBRANTE_PDK_VERSION
                                                          STREQUAL 5.1.15.2))
      set(CUDNN_VERSION 7.6.6.106)
      set(DIT_VERSION_STR
          06_02_00_03
          CACHE INTERNAL "")
      set(RESOURCES_VERSION trt_06_02_00_03)
      set(NVDLA_VERSION 01_04_00)
      set(REQUIRED_CUDA_VERSION 10.2)
      message(
        STATUS "Building with TRT version ${TRT_VERSION} and cudnn version "
               "${CUDNN_VERSION}, pdk version ${VIBRANTE_PDK_VERSION}")

      # Support CI on stage-main branch
    elseif(TRT_VERSION STREQUAL "6.2.0.3" AND VIBRANTE_PDK_VERSION STREQUAL
                                              6.0.0.0)
      set(CUDNN_VERSION 7.6.6.106)
      set(DIT_VERSION_STR
          06_02_00_03
          CACHE INTERNAL "")
      set(RESOURCES_VERSION trt_06_02_00_03)
      set(NVDLA_VERSION 01_04_00)
      set(REQUIRED_CUDA_VERSION 10.2)
      message(
        STATUS "Building with TRT version ${TRT_VERSION} and cudnn version "
               "${CUDNN_VERSION}, pdk version ${VIBRANTE_PDK_VERSION}")

    elseif(TRT_VERSION STREQUAL "6.3.1.3" AND VIBRANTE_PDK_VERSION STREQUAL
                                              5.2.0.0)
      set(CUDNN_VERSION 7.6.6.184)
      set(DIT_VERSION_STR
          06_03_01_03
          CACHE INTERNAL "")
      set(RESOURCES_VERSION trt_06_03_01_03)
      set(NVDLA_VERSION 01_04_00)
      set(REQUIRED_CUDA_VERSION 10.2)
      message(
        STATUS "Building with TRT version ${TRT_VERSION} and cudnn version "
               "${CUDNN_VERSION}, pdk version ${VIBRANTE_PDK_VERSION}")
    else()
      message(FATAL_ERROR "Combination of TRT version ${TRT_VERSION} and "
                          "PDK version ${VIBRANTE_PDK_VERSION} not supported")
    endif()
  else()
    if(TRT_VERSION STREQUAL "6.0.0.11")
      set(CUDNN_VERSION 7.6.3.8)
      set(DIT_VERSION_STR
          06_00_00_11
          CACHE INTERNAL "")
      set(RESOURCES_VERSION trt_06_00_00_11)
      set(NVDLA_VERSION 01_04_00)
      set(REQUIRED_CUDA_VERSION 10.2)
      message(STATUS "Building with TRT version ${TRT_VERSION} and "
                     "cudnn version ${CUDNN_VERSION}")

      # Support default gvs build
    elseif(TRT_VERSION STREQUAL "6.2.0.3")
      set(CUDNN_VERSION 7.6.6.106)
      set(DIT_VERSION_STR
          06_02_00_03
          CACHE INTERNAL "")
      set(RESOURCES_VERSION trt_06_02_00_03)
      set(NVDLA_VERSION 01_04_00)
      set(REQUIRED_CUDA_VERSION 10.2)
      message(STATUS "Building with TRT version ${TRT_VERSION} and "
                     "cudnn version ${CUDNN_VERSION}")

    elseif(TRT_VERSION STREQUAL "6.3.1.3")
      set(CUDNN_VERSION 7.6.6.184)
      set(DIT_VERSION_STR
          06_03_01_03
          CACHE INTERNAL "")
      set(RESOURCES_VERSION trt_06_03_01_03)
      set(NVDLA_VERSION 01_04_00)
      set(REQUIRED_CUDA_VERSION 10.2)
      message(STATUS "Building with TRT version ${TRT_VERSION} and "
                     "cudnn version ${CUDNN_VERSION}")

    elseif(TRT_VERSION STREQUAL "7.1.0.5")
      set(CUDNN_VERSION 8.0.0.0)
      set(DIT_VERSION_STR
          07_01_00_05
          CACHE INTERNAL "")
      set(RESOURCES_VERSION trt_07_01_00_05)
      set(NVDLA_VERSION 01_04_00)
      set(REQUIRED_CUDA_VERSION 11.0)
      message(STATUS "Building with TRT version ${TRT_VERSION} and "
                     "cudnn version ${CUDNN_VERSION}")
    else()
      message(FATAL_ERROR "TRT version ${TRT_VERSION} is not supported")
    endif()
  endif()
else()
  if(VIBRANTE_BUILD)
    if(VIBRANTE_PDK_VERSION STREQUAL 5.1.12.0)
      set(CUDNN_VERSION 7.6.3.8)
      set(DIT_VERSION_STR
          06_00_00_11
          CACHE INTERNAL "")
      set(RESOURCES_VERSION trt_06_00_00_11)
      set(NVDLA_VERSION 01_04_00)
      set(REQUIRED_CUDA_VERSION 10.2)
      message(STATUS "Default - Building with TRT version 6.0.0.11 and "
                     "cudnn version ${CUDNN_VERSION}")
    elseif(VIBRANTE_PDK_VERSION STREQUAL 5.1.15.0 OR VIBRANTE_PDK_VERSION
                                                     STREQUAL 5.1.15.2)
      set(CUDNN_VERSION 7.6.6.106)
      set(DIT_VERSION_STR
          06_02_00_03
          CACHE INTERNAL "")
      set(RESOURCES_VERSION trt_06_02_00_03)
      set(NVDLA_VERSION 01_04_00)
      set(REQUIRED_CUDA_VERSION 10.2)
      message(STATUS "Default - Building with TRT version 6.2.0.3 and "
                     "cudnn version ${CUDNN_VERSION}")
    elseif(VIBRANTE_PDK_VERSION STREQUAL 5.2.0.0)
      set(CUDNN_VERSION 7.6.6.184)
      set(DIT_VERSION_STR
          06_03_01_03
          CACHE INTERNAL "")
      set(RESOURCES_VERSION trt_06_03_01_03)
      set(NVDLA_VERSION 01_04_00)
      set(REQUIRED_CUDA_VERSION 10.2)
      message(STATUS "Default - Building with TRT version 6.3.1.3 and "
                     "cudnn version ${CUDNN_VERSION}")
    elseif(VIBRANTE_PDK_VERSION STREQUAL 6.0.0.0)
      set(CUDNN_VERSION 7.6.6.106)
      set(DIT_VERSION_STR
          06_02_00_03
          CACHE INTERNAL "")
      set(RESOURCES_VERSION trt_06_02_00_03)
      set(NVDLA_VERSION 01_04_00)
      set(REQUIRED_CUDA_VERSION 10.2)
      message(STATUS "Default - Building with TRT version 6.2.0.3 and "
                     "cudnn version ${CUDNN_VERSION}")
    endif()
  else()
    set(CUDNN_VERSION 7.6.6.106)
    set(DIT_VERSION_STR
        06_02_00_03
        CACHE INTERNAL "")
    set(RESOURCES_VERSION trt_06_02_00_03)
    set(NVDLA_VERSION 01_04_00)
    set(REQUIRED_CUDA_VERSION 10.2)
    message(STATUS "Default - Building with TRT version 6.2.0.3 and "
                   "cudnn version ${CUDNN_VERSION}")
  endif()
endif()

# Remove the prefix "DIT_" and create a list
string(REGEX REPLACE "DIT_" "" TRT_VERSION_ALL ${DIT_VERSION_STR})
string(REGEX REPLACE "_" ";" TRT_VERSION_ALL ${TRT_VERSION_ALL})
# Extract TRT sub versions
list(GET TRT_VERSION_ALL 0 TRT_MAJOR)
list(GET TRT_VERSION_ALL 1 TRT_MINOR)
list(GET TRT_VERSION_ALL 2 TRT_PATCH)
list(GET TRT_VERSION_ALL 3 TRT_BUILD)

math(
  EXPR
  TRT_VERSION_DECIMAL
  "${TRT_MAJOR} * 1000000 + \
                               ${TRT_MINOR} * 10000 + \
                               ${TRT_PATCH} * 100 + \
                               ${TRT_BUILD}")

string(REGEX REPLACE "_" ";" NVDLA_VERSION_ALL ${NVDLA_VERSION})
list(GET NVDLA_VERSION_ALL 0 NVDLA_MAJOR)
list(GET NVDLA_VERSION_ALL 1 NVDLA_MINOR)
list(GET NVDLA_VERSION_ALL 2 NVDLA_PATCH)

math(EXPR NVDLA_VERSION_DECIMAL "${NVDLA_MAJOR} * 10000 + \
                                 ${NVDLA_MINOR} * 100 + \
                                 ${NVDLA_PATCH}")

# -------------------------------------------------------------------------------
# Set resources folder and filename
# -------------------------------------------------------------------------------
set(DW_RESOURCES_PARENT_DIR "resources")
set(DW_RESOURCES_DATA_DIR "${RESOURCES_VERSION}")
set(DW_RESOURCES_WEIGHTS_DIR "weights")
set(DW_RESOURCES_OUSTER_LUT_DIR "ouster_lut")
set(DW_RESOURCES_PAK_NAME "resources.pak")
