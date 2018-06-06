#.rst:
# FindVisage
# ----------
#
# Try to find Visage (Face Tracking)
#
# Once done this will define::
#
#   Visage_FOUND          - True if Visage was found
#   Visage_INCLUDE_DIRS   - include directories for Visage
#   Visage_LIBRARIES      - link against this library to use Visage
#
# The module will also define two cache variables::
#
#   Visage_INCLUDE_DIR    - the Visage include directory
#   Visage_LIBRARY        - the path to the Visage library
#

#=============================================================================
# Copyright 2014 TUM FAR
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# (To distribute this file outside of CMake, substitute the full
#  License text for the above reference.)

find_path(Visage_INCLUDE_DIR
  NAMES
    visageVision.h
  PATHS
    ENV "PROGRAMFILES(X86)"
  PATH_SUFFIXES
    "Visage Technologies/visageSDK/include")

if(WIN32)
  if(CMAKE_SIZEOF_VOID_P EQUAL 4)
    find_library(Visage_LIBRARY
      NAMES libVisageVision
      PATHS
        ENV "PROGRAMFILES(X86)"
      PATH_SUFFIXES
        "Visage Technologies/visageSDK/lib")
  elseif(CMAKE_SIZEOF_VOID_P EQUAL 8)
    find_library(Visage_LIBRARY
      NAMES libVisageVision64
      PATHS
        ENV "PROGRAMFILES(X86)"
      PATH_SUFFIXES
        "Visage Technologies/visageSDK/lib")
  endif()
else()
  find_library(Visage_LIBRARY
    NAMES libVisageVision)
endif()

set(Visage_LIBRARIES ${Visage_LIBRARY})
set(Visage_INCLUDE_DIRS ${Visage_INCLUDE_DIR})

find_package_handle_standard_args(
  Visage
  FOUND_VAR Visage_FOUND
  REQUIRED_VARS Visage_LIBRARY Visage_INCLUDE_DIR)
  #VERSION_VAR Visage_VERSION_STRING)

mark_as_advanced(
  Visage_INCLUDE_DIR
  Visage_LIBRARY)