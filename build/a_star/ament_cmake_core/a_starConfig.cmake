# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_a_star_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED a_star_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(a_star_FOUND FALSE)
  elseif(NOT a_star_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(a_star_FOUND FALSE)
  endif()
  return()
endif()
set(_a_star_CONFIG_INCLUDED TRUE)

# output package information
if(NOT a_star_FIND_QUIETLY)
  message(STATUS "Found a_star: 0.0.0 (${a_star_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'a_star' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${a_star_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(a_star_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake;ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake")
foreach(_extra ${_extras})
  include("${a_star_DIR}/${_extra}")
endforeach()
