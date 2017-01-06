# Module for locating the jsoncpp library.
#
# Add this file in your project before using it in a subfolder cmake/Modules and then
# add this in your project :
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
#
# You can then call find_package(JsonCpp REQUIRED) to find this library on your system.
#
# Customizable variables:
# JSONCPP_ROOT_DIR
# This variable points to the JSONCPP installation root directory. On Windows the
# library location typically will have to be provided explicitly using the
# -D command-line option. The directory should include the include/json,
# lib and/or bin sub-directories. This can be get on any system with a make install
# of the library which will result in a system folder with the right structure :
# someinstalldir/lib
# someinstalldir/include
#
# If someinstalldir is on the Path on windows, one of /usr/ /usr/local on linux CMake
# will find it thanks to this. (If using MinGW I advice specifying MinGW folder as
# install prefix.
#
# Read-only variables:
# JSONCPP_FOUND
# Indicates whether the library has been found.
#
# JSONCPP_INCLUDE_DIRS
# Points to the jsoncpp include directory.
#
# JSONCPP_LIBRARIES
# Points to the jsoncpp libraries that should be passed to
# target_link_libararies.
#
# Usage examples
# find_package(JsonCpp REQUIRED)
# find_package(JsonCpp 0.5.0)
#

INCLUDE (FindPackageHandleStandardArgs)

FIND_PATH (JSONCPP_ROOT_DIR
 NAMES json/json.h include/json/json.h include/jsoncpp/json/json.h
 PATHS ENV JSONCPP_ROOT_DIR
 HINTS /usr/local /usr
 DOC "jsoncpp root directory")

# Re-use the previous path:
FIND_PATH (JSONCPP_INCLUDE_DIR
 NAMES json/json.h
 HINTS ${JSONCPP_ROOT_DIR} ${JSONCPP_ROOT_DIR}/include/jsoncpp
 PATH_SUFFIXES include
 DOC "jsoncpp include directory")

FIND_LIBRARY (JSONCPP_LIBRARY_RELEASE
 NAMES libjsoncpp.a jsoncpp
 HINTS ${JSONCPP_ROOT_DIR} ${JSONCPP_ROOT_DIR}/lib
 PATH_SUFFIXES ${_JSONCPP_POSSIBLE_LIB_SUFFIXES}
 DOC "jsoncpp release library")

IF (NOT DEFINED JSONCPP_LIBRARIES)
 IF (JSONCPP_LIBRARY_RELEASE)
 SET (JSONCPP_LIBRARY ${JSONCPP_LIBRARY_RELEASE} CACHE DOC
 "jsoncpp library")
 ENDIF (JSONCPP_LIBRARY_RELEASE)
ENDIF (NOT DEFINED JSONCPP_LIBRARIES)

SET (JSONCPP_INCLUDE_DIRS ${JSONCPP_INCLUDE_DIR})
SET (JSONCPP_LIBRARIES ${JSONCPP_LIBRARY})

MARK_AS_ADVANCED (JSONCPP_ROOT_DIR JSONCPP_INCLUDE_DIR JSONCPP_LIBRARY
 JSONCPP_LIBRARY_DEBUG JSONCPP_LIBRARY_RELEASE)

FIND_PACKAGE_HANDLE_STANDARD_ARGS (jsoncpp REQUIRED_VARS JSONCPP_ROOT_DIR
 JSONCPP_INCLUDE_DIR JSONCPP_LIBRARY VERSION_VAR JSONCPP_VERSION)
