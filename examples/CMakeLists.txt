cmake_minimum_required (VERSION 3.8.0)
project (zed-openni-examples)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/OpenNI2/Drivers)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/OpenNI2/Drivers)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})


include_directories(${OPENNI2_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Common/)

########################################################################
### Common Code
FILE(GLOB_RECURSE COMMON_HDR_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/Common/*.h ${CMAKE_CURRENT_SOURCE_DIR}/Common/*.hpp
)
FILE(GLOB_RECURSE COMMON_SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/Common/*.c ${CMAKE_CURRENT_SOURCE_DIR}/Common/*.cpp
)

########################################################################
### ZED Simple Viewer
set(EX1_NAME ZEDSimpleViewer)

FILE(GLOB_RECURSE EX1_HDR_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/${EX1_NAME}/*.h ${CMAKE_CURRENT_SOURCE_DIR}/${EX1_NAME}/*.hpp
)

FILE(GLOB_RECURSE EX1_SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/${EX1_NAME}/*.c ${CMAKE_CURRENT_SOURCE_DIR}/${EX1_NAME}/*.cpp
)

add_executable(${EX1_NAME}
    ${EX1_HDR_FILES}
    ${EX1_SRC_FILES}
    ${COMMON_HDR_FILES}
    ${COMMON_SRC_FILES}
)
SET(CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS} "-no-pie")
target_link_libraries(${EX1_NAME}
    ${OPENNI2_LIBRARIES}
    glut GL
)
