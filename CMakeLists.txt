cmake_minimum_required(VERSION 2.8)
include(FindPkgConfig)
include(UsePkgConfig)

# --- general project setup ---
project("RoboticsCoordinateTransform")
set(RCT_NAME             rct)
set(RCT_NAME_RSB         rctrsb)
set(RCT_NAME_ROS         rctros)
set(RCT_NAME_PROTO       rctproto)
set(RCT_VERSION_MAJOR    "1")
set(RCT_VERSION_MINOR    "0")
set(RCT_VERSION_REVISION "0")
set(RCT_VERSION_SHORT    "${RCT_VERSION_MAJOR}.${RCT_VERSION_MINOR}")
set(RCT_API_VERSION      "${RCT_VERSION_MAJOR}.${RCT_VERSION_MINOR}")
set(RCT_VERSION          "${RCT_VERSION_MAJOR}.${RCT_VERSION_MINOR}.${RCT_VERSION_REVISION}")
set(RCT_PACKAGE          "${RCT_NAME}-${RCT_VERSION_MAJOR}.${RCT_VERSION_MINOR}")
math(EXPR RCT_VERSION_MAJOR_CONT     "${RCT_VERSION_MAJOR}*1000000")
math(EXPR RCT_VERSION_MINOR_CONT     "${RCT_VERSION_MINOR}*1000")
math(EXPR RCT_VERSION_REVISION_CONT  "${RCT_VERSION_REVISION}*1")
math(EXPR RCT_VERSION_CONT           "${RCT_VERSION_MAJOR_CONT}+${RCT_VERSION_MINOR_CONT}+${RCT_VERSION_REVISION_CONT}")

set(PREFIX     ${CMAKE_INSTALL_PREFIX})
set(BINDIR     "${PREFIX}/bin")
set(INCLUDEDIR "${PREFIX}/include")
set(MANDIR     "${PREFIX}/man")
set(LIBDIR     "${PREFIX}/lib/")
set(DATADIR    "${PREFIX}/share/${RCT_PACKAGE}")

option(BUILD_EXAMPLES "build examples?" ON)

if(WIN32)
    set(RCT_BUILD_TYPE      STATIC)
	set(RCT_BUILD_PREFIX    ${CMAKE_STATIC_LIBRARY_PREFIX})
	set(RCT_BUILD_SUFFIX    ${CMAKE_STATIC_LIBRARY_SUFFIX})
else()
    set(RCT_BUILD_TYPE      SHARED)
	set(RCT_BUILD_PREFIX    ${CMAKE_SHARED_LIBRARY_PREFIX})
	set(RCT_BUILD_SUFFIX    ${CMAKE_SHARED_LIBRARY_SUFFIX})
endif()


set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

# --- dependencies ---
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)
# must be done before rsc, because rsc module for finding eigen is broken!
find_package(Eigen3 REQUIRED)
include_directories(BEFORE SYSTEM  ${Eigen3_INCLUDE_DIRS})
link_directories(${Eigen3_LIBRARY_DIRS})

#rsc
find_package(RSC 0.11 REQUIRED)
message(STATUS "RSC version: ${RSC_VERSION}")
message(STATUS "RSC libraries: ${RSC_LIBRARIES}")
message(STATUS "RSC runtime library dirs: ${RSC_RUNTIME_LIBRARY_DIRS}")
message(STATUS "RSC CMake module path: ${RSC_CMAKE_MODULE_PATH}")
include_directories(BEFORE SYSTEM ${RSC_INCLUDE_DIRS})
link_directories(${RSC_LIBRARY_DIRS})

find_package(RSC-CMake 0.11 REQUIRED)
list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_INSTALL_PREFIX}/share/cmake/Modules" ${RSC_CMAKE_MODULE_PATH})

message(STATUS "Using CMake module path: ${CMAKE_MODULE_PATH}")

find_package(ProtocolBuffers REQUIRED)
if(NOT PROTOBUF_LIBRARY)
    message(FATAL_ERROR "protobuf library not found")
endif()
if(NOT PROTOBUF_INCLUDE_DIR)
    message(FATAL_ERROR "protobuf include dir not found")
endif()
if(NOT PROTOBUF_PROTOC_EXECUTABLE)
    message(FATAL_ERROR "protoc executable not found")
endif()

find_package(tf2 QUIET REQUIRED)
include_directories(BEFORE SYSTEM  ${tf2_INCLUDE_DIRS})
link_directories(${tf2_LIBRARY_DIRS})
if(tf2_FOUND)
	message(STATUS "tf2 version: ${tf2_VERSION}")
	set(RCT_HAVE_TF2 TRUE)
else(tf2_FOUND)
	message(STATUS "tf2 not found. Support will NOT be built!")
endif(tf2_FOUND)

#rsb/t
find_package(RSB 0.11 QUIET)
find_package(RST 0.11 QUIET)
if(RSB_FOUND AND RST_FOUND)
	message(STATUS "RSB version: ${RSB_VERSION}")
	message(STATUS "RST version: ${RST_VERSION}")
	set(RCT_HAVE_RSB TRUE)
	include_directories(BEFORE SYSTEM  ${RSB_INCLUDE_DIRS} ${RST_INCLUDE_DIRS})
	link_directories(${RSB_LIBRARY_DIRS})
	add_definitions(${RST_CFLAGS})
else(RSB_FOUND AND RST_FOUND)
	message(STATUS "RSB/RST not found. Support will NOT be built!")
endif(RSB_FOUND AND RST_FOUND)

#boost
find_package(Boost REQUIRED QUIET COMPONENTS thread program_options date_time)
include_directories(${Boost_INCLUDE_DIRS})
add_definitions(-DBOOST_LOG_DYN_LINK)

configure_file(${CMAKE_SOURCE_DIR}/core/src/rct/rctConfig.h.in ${CMAKE_BINARY_DIR}/rct/rctConfig.h)

PROTOBUF_GENERATE(CPP PROTO_CPP_SOURCES PROTO_CPP_HEADERS
                      PROTOFILES ${CMAKE_SOURCE_DIR}/rsb/src/rct/proto/FrameTransform.proto
                      PROTOROOT ${CMAKE_SOURCE_DIR}/rsb/src/rct/proto/
                      INCLUDES ${RST_STABLE_PROTO_ROOT}
                      OUTPATH ${CMAKE_BINARY_DIR}/rct/)

include_directories(${CMAKE_BINARY_DIR})

# --- build instructions proto ---
ADD_LIBRARY(${RCT_NAME_PROTO} SHARED ${PROTO_CPP_SOURCES} ${PROTO_CPP_HEADERS})
TARGET_LINK_LIBRARIES(${RCT_NAME_PROTO} ${RST_LIBRARIES})
SET_TARGET_PROPERTIES(${RCT_NAME_PROTO} PROPERTIES VERSION ${RCT_VERSION} SOVERSION ${RCT_API_VERSION})
INSTALL(TARGETS ${RCT_NAME_PROTO} LIBRARY DESTINATION ${LIBDIR})

# --- build instructions modules ---
add_subdirectory(rsb/src)
add_subdirectory(core/src)
add_subdirectory(ros/src)
if(BUILD_EXAMPLES)
	ADD_SUBDIRECTORY(examples/src)
endif(BUILD_EXAMPLES)

configure_file(RCTConfig.cmake.in "${PROJECT_BINARY_DIR}/RCTConfig.cmake" @ONLY)
configure_file(RCTConfigVersion.cmake.in "${PROJECT_BINARY_DIR}/RCTConfigVersion.cmake" @ONLY)

# Install the PCAConfig.cmake and PCAConfigVersion.cmake
install(FILES
  "${PROJECT_BINARY_DIR}/RCTConfig.cmake"
  "${PROJECT_BINARY_DIR}/RCTConfigVersion.cmake"
  DESTINATION "${DATADIR}")
