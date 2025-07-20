# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_nmpc_core_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED nmpc_core_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(nmpc_core_FOUND FALSE)
  elseif(NOT nmpc_core_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(nmpc_core_FOUND FALSE)
  endif()
  return()
endif()
set(_nmpc_core_CONFIG_INCLUDED TRUE)

# output package information
if(NOT nmpc_core_FIND_QUIETLY)
  message(STATUS "Found nmpc_core: 0.0.0 (${nmpc_core_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'nmpc_core' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${nmpc_core_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(nmpc_core_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${nmpc_core_DIR}/${_extra}")
endforeach()
