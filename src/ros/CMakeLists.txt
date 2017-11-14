cmake_minimum_required(VERSION 2.8.3)
project( sptam )

## Enable most warnings
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations -std=c++11")

## Enable C++11 support
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## maximize optimization
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O4")

if( USE_LOOPCLOSURE )
  # DLoopDetector library (must be properly installed from github repos)
  find_package(DLib REQUIRED)
  find_package(DBoW2 REQUIRED)
  find_package(DLoopDetector REQUIRED)
  include_directories(${DLib_INCLUDE_DIRS} ${DBoW2_INCLUDE_DIRS} ${DLoopDetector_INCLUDE_DIRS})
  set(DLD_LIBRARIES ${DLib_LIBRARIES} ${DBoW2_LIBRARIES})

  # Find OpenGV
  find_package(OpenGV REQUIRED)
  INCLUDE_DIRECTORIES(${OPENGV_INCLUDE_DIR})

  # List of files to compile LoopClosing module
  FILE(GLOB LC_SRC ../sptam/loopclosing/*.cpp ../sptam/loopclosing/detectors/*.cpp)

  # If USE_LOOPCLOSURE isnt set, then LC_SRC will be "blank"
endif()

# TODO check for SPTAM_LIBRARIES to be defined

add_library(sptam_nodelet SPTAMNodelet.cpp SPTAMInterface.cpp ${LC_SRC})
target_link_libraries(sptam_nodelet ${SPTAM_LIBRARIES})

add_executable(sptam_node sptam_node.cpp)
target_link_libraries( sptam_node ${SPTAM_LIBRARIES})

################################################
## Inicio - Agregado
################################################

# Tracker Node
add_executable( tracker_node tracker_node.cpp)
add_dependencies( tracker_node sptam_generate_messages_cpp sptam_generate_messages_py)
target_link_libraries( tracker_node ${SPTAM_LIBRARIES})
# Linkeo además con la librería creada
#target_link_libraries( tracker_node sptam_library)

# Map Node
add_executable( map_node map_node.cpp)
add_dependencies( map_node sptam_generate_messages_cpp sptam_generate_messages_py)
target_link_libraries( map_node ${SPTAM_LIBRARIES})
#target_link_libraries( map_node sptam_library)


string(ASCII 27 Esc)
set(ColourReset "${Esc}[m")
set(ColourBold  "${Esc}[1m")
set(Red         "${Esc}[31m")
set(Green       "${Esc}[32m")
set(Yellow      "${Esc}[33m")
set(Blue        "${Esc}[34m")
set(Magenta     "${Esc}[35m")
set(Cyan        "${Esc}[36m")
set(White       "${Esc}[37m")
set(BoldRed     "${Esc}[1;31m")
set(BoldGreen   "${Esc}[1;32m")
set(BoldYellow  "${Esc}[1;33m")
set(BoldBlue    "${Esc}[1;34m")
set(BoldMagenta "${Esc}[1;35m")
set(BoldCyan    "${Esc}[1;36m")
set(BoldWhite   "${Esc}[1;37m")


FIND_PACKAGE(OpenCV REQUIRED)

# Busco una librería
FIND_LIBRARY(OPENCV NAMES opencv_core)

MESSAGE("${BoldYellow}-- Project:")
MESSAGE(${BoldGreen}${PROJECT_NAME})

# Imprimo librerías cargadas
MESSAGE("${BoldYellow}-- Librerias cargadas:")
MESSAGE(${BoldGreen}${OpenCV_LIBS})


MESSAGE("${BoldYellow}-- Target Project:")
MESSAGE(${BoldGreen}${${PROJECT_NAME}_EXPORTED_TARGETS})

MESSAGE("${BoldYellow}-- Se encontró la ibrería:${BoldGreen}${G2O_LIBRARIES}")

MESSAGE("${BoldYellow}-- Directorios Incluidos:${BoldGreen}${${PROJECT_NAME}_INCLUDE_DIRS} ")
MESSAGE("${BoldYellow}-- Directorios Catkin Incluidos:${BoldGreen}${catkin_INCLUDE_DIRS} ")

################################################
## Fin - Agregado
################################################