set(CMAKE_SYSTEM_NAME "Linux")
set(CMAKE_SYSTEM_VERSION 1)
set(VIBRANTE_BUILD ON)       #flags for the CMakeList.txt
set(VIBRANTE ON)       #flags for the CMakeList.txt
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# --------------------------------------------
# Set VIBRANTE version numbers
# --------------------------------------------

# parse vibrante branch / GCID information from PDK version files
if(((NOT DEFINED VIBRANTE_PDK_BRANCH) OR (NOT DEFINED VIBRANTE_PDK_GCID)) AND VIBRANTE_PDK)
    if(EXISTS "${VIBRANTE_PDK}/lib-target/version-nv-pdk.txt")
        set(VIBRANTE_PDK_FILE "${VIBRANTE_PDK}/lib-target/version-nv-pdk.txt")
    elseif(EXISTS "${VIBRANTE_PDK}/lib-target/version-nv-sdk.txt")
        set(VIBRANTE_PDK_FILE "${VIBRANTE_PDK}/lib-target/version-nv-sdk.txt")
    endif()

    if(VIBRANTE_PDK_FILE)
       file(READ ${VIBRANTE_PDK_FILE} version-nv-pdk)
    else()
       message(FATAL_ERROR "Can't open ${VIBRANTE_PDK}/lib-target/version-nv-(pdk/sdk).txt for PDK branch detection")
    endif()

    # Get branch from version-nv-pdk
    if(NOT DEFINED VIBRANTE_PDK_BRANCH)
      if(${version-nv-pdk} MATCHES "^(.+)-[0123456789]+")
          set(VIBRANTE_PDK_BRANCH ${CMAKE_MATCH_1} CACHE STRING "Cross-compilation PDK branch name")
          message(STATUS "VIBRANTE_PDK_BRANCH = ${VIBRANTE_PDK_BRANCH}")
      else()
          message(FATAL_ERROR "Can't determine PDK branch for PDK ${VIBRANTE_PDK}")
      endif()
    endif()

    # Get GCID from version-nv-pdk. Expecting version format to be: "version-gcid"
    if(NOT DEFINED VIBRANTE_PDK_GCID)
        string(REPLACE "-" ";" PDK_VERSION_LIST ${version-nv-pdk})
        list(LENGTH PDK_VERSION_LIST _PDK_VERSION_LIST_LENGTH)
        if (NOT _PDK_VERSION_LIST_LENGTH EQUAL 2)
            message(FATAL_ERROR "Unsupported pdk version string detected. Unable to set VIBRANTE_PDK_GCID. version-nv-pdk is ${version-nv-pdk}")
        endif()
        list(GET PDK_VERSION_LIST 1 VIBRANTE_PDK_GCID)
        string(STRIP ${VIBRANTE_PDK_GCID} VIBRANTE_PDK_GCID)
    endif()
endif()

if(DEFINED VIBRANTE_PDK_BRANCH)
  string(REPLACE "." ";" PDK_VERSION_LIST ${VIBRANTE_PDK_BRANCH})

  # Some PDK's have less than three version numbers. Pad the list so we always
  # have at least three numbers, allowing pre-existing logic depending on major,
  # minor, patch versioning to work without modifications
  list(LENGTH PDK_VERSION_LIST _PDK_VERSION_LIST_LENGTH)
  while(_PDK_VERSION_LIST_LENGTH LESS 3)
    list(APPEND PDK_VERSION_LIST 0)
    math(EXPR _PDK_VERSION_LIST_LENGTH "${_PDK_VERSION_LIST_LENGTH} + 1")
  endwhile()

  set(VIBRANTE_PDK_PATCH 0)
  set(VIBRANTE_PDK_BUILD 0)

  list(LENGTH PDK_VERSION_LIST PDK_VERSION_LIST_LENGTH)

  list(GET PDK_VERSION_LIST 0 VIBRANTE_PDK_MAJOR)
  list(GET PDK_VERSION_LIST 1 VIBRANTE_PDK_MINOR)
  if(PDK_VERSION_LIST_LENGTH GREATER 2)
    list(GET PDK_VERSION_LIST 2 VIBRANTE_PDK_PATCH)
  endif()

  if(PDK_VERSION_LIST_LENGTH GREATER 3)
    list(GET PDK_VERSION_LIST 3 VIBRANTE_PDK_BUILD)
  endif()

  set(VIBRANTE_PDK_VERSION ${VIBRANTE_PDK_MAJOR}.${VIBRANTE_PDK_MINOR}.${VIBRANTE_PDK_PATCH}.${VIBRANTE_PDK_BUILD})

  add_definitions(-DVIBRANTE_PDK_VERSION=\"${VIBRANTE_PDK_VERSION}\") # requires escaping so it is treated as a string
                                                                      # and not an invalid floating point with too many decimal points
  add_definitions(-DVIBRANTE_PDK_MAJOR=${VIBRANTE_PDK_MAJOR})
  add_definitions(-DVIBRANTE_PDK_MINOR=${VIBRANTE_PDK_MINOR})
  add_definitions(-DVIBRANTE_PDK_PATCH=${VIBRANTE_PDK_PATCH})
  add_definitions(-DVIBRANTE_PDK_BUILD=${VIBRANTE_PDK_BUILD})

  math(EXPR VIBRANTE_PDK_DECIMAL "${VIBRANTE_PDK_MAJOR} * 1000000 + \
                                  ${VIBRANTE_PDK_MINOR} * 10000 + \
                                  ${VIBRANTE_PDK_PATCH} * 100 + \
                                  ${VIBRANTE_PDK_BUILD}")
  add_definitions(-DVIBRANTE_PDK_DECIMAL=${VIBRANTE_PDK_DECIMAL})

  message(STATUS "Vibrante version ${VIBRANTE_PDK_VERSION}")
endif()


# --------------------------------------------
# END Set VIBRANTE version numbers
# --------------------------------------------


# Specify the cross compiler
set(TOOLCHAIN "$ENV{HOME}/nvidia/nvidia_sdk/DRIVE_OS_5.2.0_SDK_Linux_OS_DRIVE_AGX_XAVIER/DRIVEOS/toolchains/gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu") 
set(CMAKE_CXX_COMPILER "${TOOLCHAIN}/bin/aarch64-linux-gnu-g++") 
set(CMAKE_C_COMPILER "${TOOLCHAIN}/bin/aarch64-linux-gnu-gcc") 

# Targetfs path 
set(ROS_SYSROOT "$ENV{HOME}/nvidia/nvidia_sdk/DRIVE_OS_5.2.0_SDK_Linux_OS_DRIVE_AGX_XAVIER/DRIVEOS/drive-t186ref-linux/targetfs") 

# Library paths 
set(LD_PATH "${ROS_SYSROOT}/usr/lib/aarch64-linux-gnu") 
set(LD_PATH_EXTRA "${ROS_SYSROOT}/lib/aarch64-linux-gnu") 

# setup compiler for cross-compilation 
set(CMAKE_CXX_FLAGS           "-fPIC"               CACHE STRING "c++ flags") 
set(CMAKE_C_FLAGS             "-fPIC"               CACHE STRING "c flags") 
set(CMAKE_SHARED_LINKER_FLAGS ""                    CACHE STRING "shared linker flags") 
set(CMAKE_MODULE_LINKER_FLAGS ""                    CACHE STRING "module linker flags") 
set(CMAKE_EXE_LINKER_FLAGS    ""                    CACHE STRING "executable linker flags") 
set(CMAKE_FIND_ROOT_PATH ${ROS_SYSROOT} ${ROS_SYSROOT}/.. ${ROS_SYSROOT}/usr/local/ /usr/local)

# Set compiler flags 
set(CMAKE_SHARED_LINKER_FLAGS   "--sysroot=${ROS_SYSROOT} -L${LD_PATH} -L${LD_PATH_EXTRA} -Wl,-rpath,${LD_PATH} -Wl,-rpath,${LD_PATH_EXTRA} -Wl,-rpath,${LD_PATH} -Wl,-rpath,${LD_PATH_EXTRA} ${CMAKE_SHARED_LINKER_FLAGS}")
set(CMAKE_MODULE_LINKER_FLAGS   "--sysroot=${ROS_SYSROOT} -L${LD_PATH} -L${LD_PATH_EXTRA} -Wl,-rpath,${LD_PATH} -Wl,-rpath,${LD_PATH_EXTRA} -Wl,-rpath,${LD_PATH} -Wl,-rpath,${LD_PATH_EXTRA} ${CMAKE_SHARED_LINKER_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS      "--sysroot=${ROS_SYSROOT} -L${LD_PATH} -L${LD_PATH_EXTRA} -Wl,-rpath,${LD_PATH} -Wl,-rpath,${LD_PATH_EXTRA} -Wl,-rpath,${LD_PATH_EXTRA} ${CMAKE_EXE_LINKER_FLAGS}")
set(CMAKE_C_FLAGS "-fPIC --sysroot=${ROS_SYSROOT} -fpermissive -g" CACHE INTERNAL "" FORCE)
set(CMAKE_CXX_FLAGS "-fPIC --sysroot=${ROS_SYSROOT} -fpermissive -g" CACHE INTERNAL "" FORCE)

# Search for programs only in the build host directories 
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER) 

# Search for libraries and headers only in the target directories 
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY) 
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY) 

# set system default include dir
include_directories(BEFORE SYSTEM ${CUDA_DIR}/targets/aarch64-linux/include)
include_directories(BEFORE SYSTEM ${ROS_SYSROOT}/../include)
