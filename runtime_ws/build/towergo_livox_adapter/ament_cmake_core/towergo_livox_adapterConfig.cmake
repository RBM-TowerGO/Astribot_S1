# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_towergo_livox_adapter_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED towergo_livox_adapter_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(towergo_livox_adapter_FOUND FALSE)
  elseif(NOT towergo_livox_adapter_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(towergo_livox_adapter_FOUND FALSE)
  endif()
  return()
endif()
set(_towergo_livox_adapter_CONFIG_INCLUDED TRUE)

# output package information
if(NOT towergo_livox_adapter_FIND_QUIETLY)
  message(STATUS "Found towergo_livox_adapter: 0.1.0 (${towergo_livox_adapter_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'towergo_livox_adapter' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${towergo_livox_adapter_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(towergo_livox_adapter_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${towergo_livox_adapter_DIR}/${_extra}")
endforeach()
