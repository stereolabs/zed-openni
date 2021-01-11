cmake_minimum_required( VERSION 3.10 )

# Create Project
project( SimpleMultiViewer )

# Require C++14 (or later)
set( CMAKE_CXX_STANDARD 14 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_CXX_EXTENSIONS OFF )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

FILE(GLOB_RECURSE HDR_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h ${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp
)
FILE(GLOB_RECURSE SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${OPENNI2_INCLUDE_DIRS})

if(UNIX)
    SET(CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS} "-no-pie")
endif()

add_executable( ${PROJECT_NAME}
    ${COMMON_HDR_FILES}
    ${HDR_FILES}
    ${SRC_FILES}
)

# Additional Dependencies
target_link_libraries( ${PROJECT_NAME}
    ${OPENNI2_LIBRARIES}
    glut GL
)


