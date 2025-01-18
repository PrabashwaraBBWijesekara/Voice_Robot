# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_voice_control_robot_package_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED voice_control_robot_package_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(voice_control_robot_package_FOUND FALSE)
  elseif(NOT voice_control_robot_package_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(voice_control_robot_package_FOUND FALSE)
  endif()
  return()
endif()
set(_voice_control_robot_package_CONFIG_INCLUDED TRUE)

# output package information
if(NOT voice_control_robot_package_FIND_QUIETLY)
  message(STATUS "Found voice_control_robot_package: 0.0.0 (${voice_control_robot_package_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'voice_control_robot_package' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT voice_control_robot_package_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(voice_control_robot_package_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${voice_control_robot_package_DIR}/${_extra}")
endforeach()
