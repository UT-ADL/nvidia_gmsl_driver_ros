# Copyright (c) 2017-2019, NVIDIA CORPORATION.  All rights reserved.

# -------------------------------------------------------------------------------
# Dependencies
# -------------------------------------------------------------------------------

if(VIBRANTE)
  set(vibrante_DIR
      "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/${SDK_ARCH_DIR}/vibrante"
      CACHE PATH '' FORCE)
  if(NOT VIBRANTE_V5Q)

  endif()
  set(DW_USE_NVMEDIA_DRIVE ON)
  set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
  set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
else()
  # this is for parts of the code needed only on LINUX because of special care
  # needed for nvmedia x86
  set(DW_USE_NVMEDIA_X86 ON)
endif()

# Hide settings in default cmake view
mark_as_advanced(vibrante_DIR vibrante_Xlibs_DIR)
