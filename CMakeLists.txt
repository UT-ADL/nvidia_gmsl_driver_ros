cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

project(sekonix_camera_ut)

set(CMAKE_WARN_DEPRECATED OFF)
set(CMAKE_BUILD_TYPE release)


#-------------------------------------------------------------------------------
# Uncomment to force loading VIBRANTE specific files (Development only).
#-------------------------------------------------------------------------------
#add_compile_definitions(VIBRANTE)

#-------------------------------------------------------------------------------
# Configure project for cross-compilation
#-------------------------------------------------------------------------------
include(cmake/MainConfiguration.cmake)

find_package(OpenCV REQUIRED)
find_package(YAMLCPP REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        geometry_msgs
        image_transport
        camera_info_manager
        h264_image_transport_msgs
        )

include_directories(
        ${catkin_INCLUDE_DIRS}
        $ENV{HOME}/nvidia/nvidia_sdk/DRIVE_OS_5.2.0_SDK_Linux_OS_DDPX/DRIVEOS/drive-t186ref-linux/include
        $ENV{HOME}/nvidia/nvidia_sdk/DRIVE_OS_5.2.0_SDK_Linux_OS_DRIVE_AGX_XAVIER/DRIVEOS/drive-t186ref-linux/include
        include
)

if (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    include_directories(
            /usr/local/driveworks/targets/x86_64-Linux/include
    )
endif ()

catkin_package(
        LIBRARIES ${PROJECT_NAME}
)

set(NODE_NAME ${PROJECT_NAME}_node)

add_executable(${NODE_NAME}
        src/main.cpp
        src/Sekonix_driver.cpp
        src/Camera.cpp
        src/DriveworksApiWrapper.cpp
        src/framework/Log.cpp
        )

target_link_libraries(${NODE_NAME}
        ${catkin_LIBRARIES}
        nvmedia
        driveworks
        ${CUDA_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        )

install(TARGETS
        ${NODE_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY config calib launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
