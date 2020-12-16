cmake_minimum_required (VERSION 3.8.0)
project (zed-openni-driver)

set(LIB_NAME zeddriver)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_SHARED_MODULE_PREFIX "lib")
set(CMAKE_SHARED_LIBRARY_PREFIX "lib")

###########################################################################
### DEPENDENCIES
###########################################################################
# add local cmake modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

include(MacroLogFeature)

find_package(OpenNI2 REQUIRED)
macro_log_feature(OPENNI2_FOUND "OpenNI2" "Required to build ${PROJECT_NAME}" "https://github.com/OpenNI/OpenNI2" TRUE "2")
#set(OPENNI2_DIR "c:/Program Files/OpenNI2" CACHE FILEPATH "OpenNI2 SDK directory")

find_package(ZED 3)
macro_log_feature(ZED_FOUND "ZED SDK" "Required to build ${PROJECT_NAME}" "https://www.stereolabs.com" TRUE "3")
find_package(CUDA ${ZED_CUDA_VERSION} EXACT)
macro_log_feature(CUDA_FOUND "Nvidia CUDA library" "Required to build ${PROJECT_NAME}" "http://www.nvidia.com" FALSE "${ZED_CUDA_VERSION}")

macro_display_feature_log()

###########################################################################
### HEADER FILES
###########################################################################
set (HDR_FILES
    include/zedtools.hpp
    include/zedstream.hpp
    include/zeddevice.hpp
    include/zeddriver.hpp
)

###########################################################################
### SOURCE FILES
###########################################################################
set (SRC_FILES
    src/zedstream.cpp
    src/zeddevice.cpp
    src/zeddriver.cpp
)

###########################################################################
### ZED LIBS
###########################################################################
SET(ZED_LIBS
    ${ZED_LIBRARIES}
    ${CUDA_CUDA_LIBRARY}
    ${CUDA_CUDART_LIBRARY}
    ${CUDA_NPP_LIBRARIES_ZED}
)

###########################################################################
### FLAGS
###########################################################################
if(MSVC)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
    set (CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Oi /Ot /GL /GF /MD /GS- /Gy /fp:fast /arch:AVX2")
    set (CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /SUBSYSTEM:WINDOWS")
endif()

include_directories(${OPENNI2_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${LIBRARY_INSTALL_DIR})

if(UNIX)
    message(STATUS "[${LIB_NAME}] OS: Unix")
    add_definitions(-std=c++11 -Wno-deprecated-declarations -Wno-write-strings)
endif(UNIX)

###########################################################################
### Build type
###########################################################################
IF( NOT CMAKE_BUILD_TYPE )
   SET( CMAKE_BUILD_TYPE Release ... FORCE )
ENDIF()

if(CMAKE_BUILD_TYPE MATCHES Debug)
    message(STATUS "[${LIB_NAME}] Build type: Debug")
    add_definitions(-g)
elseif(CMAKE_BUILD_TYPE MATCHES Release)
    message(STATUS "[${LIB_NAME}] Build type: Release")
    add_definitions(-O2)
elseif(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
    message(STATUS "[${LIB_NAME}] Build type: Release with Debug Info")
    add_definitions(-g)
endif()

###########################################################################
### LINK
###########################################################################
add_library (${LIB_NAME} SHARED
    ${HDR_FILES}
    ${SRC_FILES}
)
target_link_libraries(${LIB_NAME}
    ${ZED_LIBS}
)

###########################################################################
### INSTALL
###########################################################################
if(NOT DRIVER_INSTALL_PATH )
    get_filename_component(_PREFIX "${OPENNI2_INCLUDE_DIRS}/../../lib/OpenNI2" REALPATH)
    set(DRIVER_INSTALL_PATH "${_PREFIX}/Drivers"
        CACHE PATH "Common prefix for all installed files " FORCE)
endif()
message(STATUS "[${LIB_NAME}] Driver install path: ${DRIVER_INSTALL_PATH}")

install(TARGETS ${LIB_NAME} LIBRARY DESTINATION ${DRIVER_INSTALL_PATH})


