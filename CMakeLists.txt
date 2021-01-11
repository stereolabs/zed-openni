cmake_minimum_required (VERSION 3.10.0)
project (zed-openni)

set(CMAKE_CXX_STANDARD 14)

option(COMPILE_EXAMPLES "Compile examples" ON)

###########################################################################
### DEPENDENCIES
###########################################################################
# add local cmake modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

include(MacroLogFeature)

set(OPENNI2_DIR "" CACHE PATH "Root of the OpenNI2 SDK directory")

message("-----------------------------------------------------------------------------")
if(EXISTS ${OPENNI2_DIR}/Include)
    message(STATUS "OpenNI2 folder: ${OPENNI2_DIR}")
    set(OPENNI2_INCLUDE_DIRS ${OPENNI2_DIR}/Include)
    message(STATUS "OpenNI2 include folder: ${OPENNI2_INCLUDE_DIRS}")
    #set(OPENNI2_LIB_DIRS ${OPENNI2_DIR}/Redist) TO BE USED WITH OCCIPITAL PRECOMPILED OpenNI2
    set(OPENNI2_LIB_DIRS ${OPENNI2_DIR}/Bin/x64-Debug) #TO BE USED WITH OCCIPITAL PRECOMPILED OpenNI2 in Debug mode
    message(STATUS "OpenNI2 libraries folder: ${OPENNI2_LIB_DIRS}")
    set(OPENNI2_LIBRARIES ${OPENNI2_LIB_DIRS}/libOpenNI2.so)
    message(STATUS "OpenNI2 library file: ${OPENNI2_LIBRARIES}")
else()
    message( FATAL_ERROR "Please set the path to OpenNI2 root folder -> '-DOPENNI2_DIR='path-to-your-openni2-installation-folder'" )
endif()
message("-----------------------------------------------------------------------------")

find_package(ZED 3)
macro_log_feature(ZED_FOUND "ZED SDK" "Required to build ${PROJECT_NAME}" "https://www.stereolabs.com" TRUE "3")
find_package(CUDA ${ZED_CUDA_VERSION} EXACT)
macro_log_feature(CUDA_FOUND "Nvidia CUDA library" "Required to build ${PROJECT_NAME}" "http://www.nvidia.com" FALSE "${ZED_CUDA_VERSION}")

macro_display_feature_log()

if(UNIX)
    message(STATUS "[${PROJECT_NAME}] OS: Unix")
    add_definitions(-std=c++14 -Wno-deprecated-declarations -Wno-write-strings -no-pie)
endif(UNIX)

if(MSVC)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++14")
endif(MSVC)

###########################################################################
### Build type
###########################################################################
IF( NOT CMAKE_BUILD_TYPE )
   SET( CMAKE_BUILD_TYPE Release ... FORCE )
ENDIF()

if(CMAKE_BUILD_TYPE MATCHES Debug)
    message(STATUS "[${PROJECT_NAME}] Build type: Debug")
    add_definitions(-g)
elseif(CMAKE_BUILD_TYPE MATCHES Release)
    message(STATUS "[${PROJECT_NAME}] Build type: Release")
    add_definitions(-O2)
elseif(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
    message(STATUS "[${PROJECT_NAME}] Build type: Release with Debug Info")
    add_definitions(-g)
endif()

###########################################################################
### Modules
###########################################################################
# Driver
add_subdirectory(driver)

#Examples
if(COMPILE_EXAMPLES)
    add_subdirectory(examples)
endif()

####################################################################
# USEFUL FILES
FILE(GLOB_RECURSE doc_files ${CMAKE_SOURCE_DIR}/*.md)
add_custom_target("DOCS" SOURCES ${doc_files})
FILE(GLOB_RECURSE script_files ${CMAKE_SOURCE_DIR}/scripts/*)
add_custom_target("SCRIPTS" SOURCES ${script_files})
####################################################################
