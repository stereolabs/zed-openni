cmake_minimum_required (VERSION 3.10.0)
project (zed-openni-examples)

message(STATUS "[${PROJECT_NAME}] Examples added")

set(CMAKE_CXX_STANDARD 14)

###########################################################################
# Example using OpenGL for rendering
###########################################################################
add_subdirectory(SimpleMultiViewer)

###########################################################################
# Examples using OpenCV for rendering
###########################################################################
set( OpenCV_DIR "C:/Program Files/opencv/build" CACHE PATH "Path to OpenCV config directory." )
find_package( OpenCV )

if(OpenCV_FOUND)
    add_subdirectory(SimpleColor)
    add_subdirectory(SimpleDepth)
    add_subdirectory(SimpleRegistered)
    add_subdirectory(SimplePointCloud)
endif()





