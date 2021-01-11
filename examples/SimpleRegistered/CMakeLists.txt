cmake_minimum_required( VERSION 3.10 )

# Create Project
project( SimpleRegistered )

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

if(UNIX)
    SET(CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS} "-no-pie")
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# OpenCV
set( OpenCV_DIR "C:/Program Files/opencv/build" CACHE PATH "Path to OpenCV config directory." )
find_package( OpenCV )

# OpenMP
find_package( OpenMP )

if( OpenMP_FOUND )
  set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}" )
  set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}" )
endif()

if( OpenCV_FOUND )
  # Additional Include Directories
  include_directories(${OPENNI2_INCLUDE_DIRS})
  include_directories( ${OpenCV_INCLUDE_DIRS} )

  add_executable( ${PROJECT_NAME}
      ${COMMON_HDR_FILES}
      ${HDR_FILES}
      ${SRC_FILES})
  # Dependencies
  target_link_libraries( ${PROJECT_NAME}
      ${OpenCV_LIBS}
      ${OPENNI2_LIBRARIES})
endif()


