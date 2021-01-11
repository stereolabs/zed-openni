cmake_minimum_required (VERSION 3.10.0)
project (zed-openni-examples)

message(STATUS "[${PROJECT_NAME}] Examples added")

set(CMAKE_CXX_STANDARD 14)

add_subdirectory(SimpleViewer)

# Using OpenCV
add_subdirectory(SimpleColor)
add_subdirectory(SimpleDepth)
add_subdirectory(SimpleRegistered)
add_subdirectory(SimplePointCloud)





