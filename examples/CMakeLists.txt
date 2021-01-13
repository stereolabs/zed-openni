cmake_minimum_required (VERSION 3.10.0)
project (zed-openni-examples)

message(STATUS "[${PROJECT_NAME}] Examples added")

set(CMAKE_CXX_STANDARD 14)

###########################################################################
# Common Files
###########################################################################
FILE(GLOB_RECURSE COMMON_HDR_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/Common/*.h ${CMAKE_CURRENT_SOURCE_DIR}/Common/*.hpp
)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Common)

###########################################################################
# Example using OpenGL for rendering
###########################################################################
add_subdirectory(SimpleMultiViewer)

###########################################################################
# Examples using OpenCV for rendering
###########################################################################
set( OpenCV_DIR "C:/devel/opencv/build" CACHE PATH "Path to OpenCV config directory." )
find_package(OpenCV)

if(UNIX AND OpenCV_FOUND)
    message("OpenCV Libraries: ${OpenCV_LIBS}")

    add_subdirectory(SimpleColor)
    add_subdirectory(SimpleDepth)
    add_subdirectory(SimpleRegistered)
    if(UNIX)
        add_subdirectory(SimplePointCloud)
    endif()
endif()




