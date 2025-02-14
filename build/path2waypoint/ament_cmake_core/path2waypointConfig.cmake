# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_path2waypoint_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED path2waypoint_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(path2waypoint_FOUND FALSE)
  elseif(NOT path2waypoint_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(path2waypoint_FOUND FALSE)
  endif()
  return()
endif()
set(_path2waypoint_CONFIG_INCLUDED TRUE)

# output package information
if(NOT path2waypoint_FIND_QUIETLY)
  message(STATUS "Found path2waypoint: 0.0.0 (${path2waypoint_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'path2waypoint' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${path2waypoint_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(path2waypoint_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${path2waypoint_DIR}/${_extra}")
endforeach()
