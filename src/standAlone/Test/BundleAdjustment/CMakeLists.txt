CMAKE_MINIMUM_REQUIRED(VERSION 2.8 FATAL_ERROR)
PROJECT(bag2o)

# set c++ compilation options ( debugging / warnings / standard )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations -std=c++11")

SET(CMAKE_BUILD_TYPE Debug)


# Include Eigen lybrary headers
INCLUDE_DIRECTORIES(/usr/include/eigen3)

SET(CHOLMOD_INCLUDE_DIR /usr/include/suitesparse/)
INCLUDE_DIRECTORIES(${CHOLMOD_INCLUDE_DIR})
# In some systems, depending on how G2O was compiled,
# the libraary suitesparseconfig has to be appended
SET(G2O_LIBRARY cholmod g2o_core g2o_stuff g2o_csparse_extension g2o_types_icp g2o_cli g2o_solver_pcg g2o_solver_eigen g2o_solver_csparse g2o_types_sba g2o_interface amd colamd lapack blas)

SET(STAM_SRC ../../../stam)
INCLUDE_DIRECTORIES(STAM_SRC)

# add core libs
FILE(GLOB types_sba_extension_files ${STAM_SRC}/Localization/types_sba_extension.cpp )

add_library( Types_sba_extension ${types_sba_extension_files})

set(TYPES_SBA_EXTENSION_LIB Types_sba_extension)


# add core libs
#FILE(GLOB localizationFiles ${STAM_SRC}/Localization/*.cpp )
#add_library( Localization ${localizationFiles})

ADD_EXECUTABLE(bag2o bag2o.cpp)

TARGET_LINK_LIBRARIES(bag2o  ${G2O_LIBRARY} ${TYPES_SBA_EXTENSION_LIB} cxsparse)



