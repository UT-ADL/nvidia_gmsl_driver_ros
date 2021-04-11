# Copyright (c) 2018-2020 NVIDIA CORPORATION.  All rights reserved.

## CMake configuration file
set(DIRS
    ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}/../lib
    ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}/../lib/stubs
    ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}/../lib64
    ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}/../lib64/stubs
)

# Find libs
find_library(cudart_LIBRARY
  NAMES cudart
  HINTS ${DIRS}
)
if(NOT cudart_LIBRARY)
    message(WARNING "Could not find cudart library. Looked in ${DIRS}")
    set(cuda-libs_FOUND False)
    return()
endif()

find_library(cuda_LIBRARY
  NAMES cuda
  HINTS ${DIRS}
)
if(NOT cuda_LIBRARY)
    message(WARNING "Could not find cuda library. Looked in ${DIRS}")
    set(cuda-libs_FOUND False)
    return()
endif()

# Interface target where all necessary libraries will be added
add_library(cuda-libs INTERFACE)
target_link_libraries(cuda-libs INTERFACE ${cudart_LIBRARY} ${cuda_LIBRARY})
target_include_directories(cuda-libs SYSTEM INTERFACE ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
