# Copyright (c) 2016-2020 NVIDIA CORPORATION.  All rights reserved.

#-------------------------------------------------------------------------------
# Build flags
#-------------------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wall -Wunused -Wunused-value -Wunused-parameter")
    set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -Werror -Wall -Wunused -Wunused-value -Wunused-parameter")
endif()

#-------------------------------------------------------------------------------
# Configured headers
#-------------------------------------------------------------------------------
include_directories(${SDK_BINARY_DIR}/configured)
