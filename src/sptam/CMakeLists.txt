cmake_minimum_required(VERSION 2.8.3)
project( sptam )

# declare cache entry parameters
SET(SHOW_POINT_CLOUD OFF)
SET(SHOW_TRACKED_FRAMES OFF CACHE BOOL "Enable/Disable OpenCV frame visualization for the tracker.")
SET(SHOW_PROFILING ON CACHE BOOL "Enable/Disable Profiling of each step.")
SET(USE_LOOPCLOSURE OFF CACHE BOOL "Enable/Disable Loopclosure feature.")
#//++++++++++++++++++++++++++++++++++++++++++++++
#// DSPTAM - new flags
SET(SHOW_PRINTS ON CACHE BOOL "Enable/Disable Prints on screen.")
#//++++++++++++++++++++++++++++++++++++++++++++++

# Enable most warnings
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations")

# Enable C++11 support
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# maximize optimization
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O4")


# Find OpenCV library
FIND_PACKAGE(OpenCV REQUIRED)
#FIND_PACKAGE(OpenCV 2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Set CMake flags in the compiler

if( SHOW_TRACKED_FRAMES )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSHOW_TRACKED_FRAMES")
endif()

if( SHOW_PROFILING )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSHOW_PROFILING")
endif()

if( USE_ODOMETRY )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_ODOMETRY")
endif()

if( USE_LOOPCLOSURE )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_LOOPCLOSURE")
endif()

#//++++++++++++++++++++++++++++++++++++++++++++++
#// DSPTAM - new flags
if( SHOW_PRINTS )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSHOW_PRINTS")
endif()
#//++++++++++++++++++++++++++++++++++++++++++++++

# List of files to compile
FILE(GLOB SLAM_SRCS *.cpp utils/*.cpp)

# Find libraries required by LoopClosing module
if( USE_LOOPCLOSURE )
  # DLoopDetector library (must be properly installed from github repos)
  find_package(DLib REQUIRED)
  find_package(DBoW2 REQUIRED)
  find_package(DLoopDetector REQUIRED)
  include_directories(${DLib_INCLUDE_DIRS} ${DBoW2_INCLUDE_DIRS} ${DLoopDetector_INCLUDE_DIRS})
  set(DLD_LIBRARIES ${DLib_LIBRARIES} ${DBoW2_LIBRARIES}) # DLoopDetector its just a header
  
  # Find OpenGV
  find_package(OpenGV REQUIRED)
  INCLUDE_DIRECTORIES(${OPENGV_INCLUDE_DIR})
  
  # List of files to compile
  FILE(GLOB LC_SRCS loopclosing/*.cpp)
  set(SLAM_SRCS ${SLAM_SRCS} ${LC_SRC})
  
  # if LC isnt defined, DLD_LIBRARIES OPENGV_LIBRARY are "blank"
endif()

# Compile the tracker visualization library
if ( SHOW_TRACKED_FRAMES )
  set(SLAM_SRCS ${SLAM_SRCS} utils/draw/Draw.cpp)
endif()

# Compile the profiling library
if ( SHOW_PROFILING )
  set(SLAM_SRCS ${SLAM_SRCS} utils/log/Logger.cpp)
endif()

# Build the sptam library
add_library( sptam  ${SLAM_SRCS} )
target_link_libraries( sptam ${OpenCV_LIBRARIES} ${DLD_LIBRARIES} ${OPENGV_LIBRARY} )


################################################
## Inicio - Agregado
################################################

add_dependencies(sptam ${catkin_EXPORTED_TARGETS})

set(LIBMESSAGE_FILTERS /opt/ros/kinetic/lib/libmessage_filters.so)
set(LIBTF2 /opt/ros/kinetic/lib/libtf2.so)
set(LIBTF2_ROS /opt/ros/kinetic/lib/libtf2_ros.so)

target_link_libraries( sptam ${OpenCV_LIBRARIES} ${DLD_LIBRARIES} ${OPENGV_LIBRARY} ${LIBMESSAGE_FILTERS} ${LIBTF2} ${LIBTF2_ROS})

################################################
## Fin - Agregado
################################################