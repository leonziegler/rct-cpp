# Eigen3 lib
#
# Script will define
#
# * Eigen3_FOUND - true if eigen was found
# * Eigen3_INCLUDE_DIRS - include directory
# * Eigen3_VERSION - version
#

if(NOT Eigen3_FIND_VERSION)
  if(NOT Eigen3_FIND_VERSION_MAJOR)
    set(Eigen3_FIND_VERSION_MAJOR 2)
  endif(NOT Eigen3_FIND_VERSION_MAJOR)
  if(NOT Eigen3_FIND_VERSION_MINOR)
    set(Eigen3_FIND_VERSION_MINOR 91)
  endif(NOT Eigen3_FIND_VERSION_MINOR)
  if(NOT Eigen3_FIND_VERSION_PATCH)
    set(Eigen3_FIND_VERSION_PATCH 0)
  endif(NOT Eigen3_FIND_VERSION_PATCH)

  set(Eigen3_FIND_VERSION "${Eigen3_FIND_VERSION_MAJOR}.${Eigen3_FIND_VERSION_MINOR}.${Eigen3_FIND_VERSION_PATCH}")
endif(NOT Eigen3_FIND_VERSION)

macro(_eigen3_check_version)
  file(READ "${Eigen3_INCLUDE_DIR}/Eigen/src/Core/util/Macros.h" _eigen3_version_header)

  string(REGEX MATCH "define[ \t]+EIGEN_WORLD_VERSION[ \t]+([0-9]+)" _eigen3_world_version_match "${_eigen3_version_header}")
  set(Eigen3_WORLD_VERSION "${CMAKE_MATCH_1}")
  string(REGEX MATCH "define[ \t]+EIGEN_MAJOR_VERSION[ \t]+([0-9]+)" _eigen3_major_version_match "${_eigen3_version_header}")
  set(Eigen3_MAJOR_VERSION "${CMAKE_MATCH_1}")
  string(REGEX MATCH "define[ \t]+EIGEN_MINOR_VERSION[ \t]+([0-9]+)" _eigen3_minor_version_match "${_eigen3_version_header}")
  set(Eigen3_MINOR_VERSION "${CMAKE_MATCH_1}")

  set(Eigen3_VERSION ${Eigen3_WORLD_VERSION}.${Eigen3_MAJOR_VERSION}.${Eigen3_MINOR_VERSION})

  if(${Eigen3_VERSION} VERSION_LESS ${Eigen3_FIND_VERSION})
    set(Eigen3_VERSION_OK FALSE)
  else(${Eigen3_VERSION} VERSION_LESS ${Eigen3_FIND_VERSION})
    set(Eigen3_VERSION_OK TRUE)
  endif(${Eigen3_VERSION} VERSION_LESS ${Eigen3_FIND_VERSION})

  if(NOT Eigen3_VERSION_OK)

    message(STATUS "Eigen3 version ${Eigen3_VERSION} found in ${Eigen3_INCLUDE_DIR}, "
                   "but at least version ${Eigen3_FIND_VERSION} is required")
  endif(NOT Eigen3_VERSION_OK)
endmacro(_eigen3_check_version)

if(Eigen3_INCLUDE_DIR)

  # in cache already
  _eigen3_check_version()
  set(Eigen3_FOUND ${Eigen3_VERSION_OK})
  include(FindPackageHandleStandardArgs)
  set(Eigen3_INCLUDE_DIRS ${Eigen3_INCLUDE_DIR})

  mark_as_advanced(Eigen3_INCLUDE_DIR)

else(Eigen3_INCLUDE_DIR)

  find_path(Eigen3_INCLUDE_DIR NAMES signature_of_eigen3_matrix_library
      HINTS ${Eigen3_ROOT}
      PATH_SUFFIXES eigen3 eigen
    )

  if(Eigen3_INCLUDE_DIR)
    _eigen3_check_version()
    set(Eigen3_FOUND ${Eigen3_VERSION_OK})
  endif(Eigen3_INCLUDE_DIR)

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(Eigen3 DEFAULT_MSG Eigen3_INCLUDE_DIR Eigen3_VERSION_OK)
  set(Eigen3_INCLUDE_DIRS ${Eigen3_INCLUDE_DIR})

  mark_as_advanced(Eigen3_INCLUDE_DIR)

endif(Eigen3_INCLUDE_DIR)