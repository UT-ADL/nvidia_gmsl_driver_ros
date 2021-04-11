# Copyright (c) 2017-2020 NVIDIA CORPORATION.  All rights reserved.

project(glew-src C CXX)

set(SOURCES
    # release-2.1.0
    src/glew.c
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Add GL/GLU link dependencies this way - don't use FindPackage(OpenGL), as this conflicts with certain
# unusal OpenGL library locations (cross-compilation, Optimus/Bumblebee laptops)
target_link_libraries(${PROJECT_NAME} PUBLIC GLU GL)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "glew")
