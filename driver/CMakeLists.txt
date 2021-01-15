cmake_minimum_required (VERSION 3.10.0)
project (zed-openni-driver)

set(LIB_NAME zeddriver)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_SHARED_MODULE_PREFIX "lib")
set(CMAKE_SHARED_LIBRARY_PREFIX "lib")

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

###########################################################################
### HEADER FILES
###########################################################################
FILE(GLOB_RECURSE HDR_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/*.h ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp
    )

###########################################################################
### SOURCE FILES
###########################################################################
FILE(GLOB_RECURSE SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/*.c ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
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

link_directories(${OPENNI2_LIB_DIRS})
link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${LIBRARY_INSTALL_DIR})

###########################################################################
### LINK
###########################################################################

add_library (${LIB_NAME} SHARED
    ${HDR_FILES}
    ${SRC_FILES}
)
target_link_libraries( ${LIB_NAME}
    ${ZED_LIBS}
)

# NOTE: the driver is automatically deployed in the Driver folder in OpenNI2
# Post-Build Event (Copy Dependencies)

add_custom_command(
    TARGET ${LIB_NAME}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
    $<TARGET_FILE:${LIB_NAME}>
    "${OPENNI2_LIB_DIRS}/OpenNI2/Drivers"
)



