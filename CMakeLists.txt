cmake_minimum_required(VERSION 2.8.3)
project( sptam )

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)

## maximize optimization
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O4")

# declare cache entry parameters
SET(SHOW_TRACKED_FRAMES ON CACHE BOOL "Enable/Disable OpenCV frame visualization for the tracker.")
SET(SHOW_PROFILING ON CACHE BOOL "Enable/Disable Profiling of each step.")
SET(USE_LOOPCLOSURE OFF CACHE BOOL "Enable/Disable Loopclosure feature.")
#SET(SHOW_PRINTS ON CACHE BOOL "Enable/Disable Prints on screen.")

if( SHOW_TRACKED_FRAMES )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSHOW_TRACKED_FRAMES")
endif()

if( SHOW_PROFILING )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSHOW_PROFILING")
endif()

if( USE_LOOPCLOSURE )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_LOOPCLOSURE")
endif()

## Find catkin macros and libraries
find_package( catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  message_filters
  message_generation
  image_geometry
  pcl_ros
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  image_transport
)

include_directories(${catkin_INCLUDE_DIRS})

################################################
## Inicio - Agregado
################################################
add_message_files( 
  FILES
  msg_kp.msg
  msg_feature.msg       
 # msg_map.msg
  msg_MapPoint.msg
  msg_KeyFrame.msg
  msg_Measurement.msg
  msg_Meas_id.msg
  msg_mapDiff.msg

)

########## RECORDAR: Para que cmake me genere la dependencia de los ejecutables con los .h generados a partir de los mensajes agregar:
       # add_dependencies(NAME_OF_EXE PACKAGE_NAME_generate_messages_cpp)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs

)



################################################
## Fin - Agregado
################################################

# Find Boost Library
find_package(Boost COMPONENTS thread system regex REQUIRED)
include_directories(${BOOST_INCLUDE_DIR})

# Find yaml-cpp Library
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

# Find OpenCV library
FIND_PACKAGE(OpenCV REQUIRED) # OpenCV 3
#FIND_PACKAGE(OpenCV 2 REQUIRED) # OpenCV 2
include_directories(${OpenCV_INCLUDE_DIRS})

# Find PCL library
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Find Suitesparse library
find_package(SuiteSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})
include_directories(${CHOLMOD_INCLUDE_DIR})

# Find G2O Library
find_package(G2O REQUIRED)
# select required components
SET(G2O_LIBRARIES ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_TYPES_SBA} ${G2O_TYPES_SLAM3D})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sptam
   CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs cv_bridge message_filters image_geometry pcl_ros nav_msgs tf2 tf2_geometry_msgs tf2_ros message_runtime
#  DEPENDS system_lib
)


###########
## Build ##
###########

# Build the sptam library
add_subdirectory( src/sptam )

set( SPTAM_LIBRARIES
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${G2O_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
  ${CHOLMOD_LIBRARIES}
  ${YamlCpp_LIBRARIES}
  cxsparse
  sptam
)

# Build the sptam node
add_subdirectory( src/ros )

#############
## Install ##
#############

#TODO

#############
## Testing ##
#############

#TODO
