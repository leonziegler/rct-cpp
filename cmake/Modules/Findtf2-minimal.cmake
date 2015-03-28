# - Try to find the libtf2 library
# Once done this will define
#
# tf2-minimal_FOUND - system has libtf2-minimal
# tf2-minimal_INCLUDE_DIRS - the libtf2-minimal include directories
# tf2-minimal_LIBRARIES - libtf2-minimal library

FIND_PATH(tf2-minimal_INCLUDE_DIR tf2/buffer_core.h
		HINTS /include /usr/include /usr/local/include
		PATH_SUFFIXES include
)
FIND_LIBRARY(tf2-minimal_LIBRARIES NAMES tf2
		HINTS /lib /usr/lib /usr/local/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(tf2-minimal DEFAULT_MSG tf2-minimal_INCLUDE_DIR tf2-minimal_LIBRARIES)
set(tf2-minimal_INCLUDE_DIRS ${tf2-minimal_INCLUDE_DIR})

MARK_AS_ADVANCED(tf2-minimal_INCLUDE_DIR tf2-minimal_LIBRARIES) 
