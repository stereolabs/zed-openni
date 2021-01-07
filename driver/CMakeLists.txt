cmake_minimum_required (VERSION 3.10.0)
project (zed-openni-driver)

set(LIB_NAME zeddriver)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_SHARED_MODULE_PREFIX "lib")
set(CMAKE_SHARED_LIBRARY_PREFIX "lib")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/OpenNI2/Drivers)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/OpenNI2/Drivers)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

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
set_target_properties(${LIB_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY_DEBUG "${OPENNI2_LIB_DIRS}/OpenNI2/Drivers/"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE "${OPENNI2_LIB_DIRS}/OpenNI2/Drivers/"
    LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO "${OPENNI2_LIB_DIRS}/OpenNI2/Drivers/"
    LIBRARY_OUTPUT_DIRECTORY_MINSIZEREL "${OPENNI2_LIB_DIRS}/OpenNI2/Drivers/"
)
target_link_libraries(${LIB_NAME}
    ${ZED_LIBS}

)

###########################################################################
### INSTALL
###########################################################################
#if(NOT DRIVER_INSTALL_PATH )
#    get_filename_component(_PREFIX "${OPENNI2_INCLUDE_DIRS}/../../lib/OpenNI2" REALPATH)
#    set(DRIVER_INSTALL_PATH "${_PREFIX}/Drivers"
#        CACHE PATH "Common prefix for all installed files " FORCE)
#endif()
#message(STATUS "[${LIB_NAME}] Driver install path: ${DRIVER_INSTALL_PATH}")

#install(TARGETS ${LIB_NAME} LIBRARY DESTINATION ${DRIVER_INSTALL_PATH})




