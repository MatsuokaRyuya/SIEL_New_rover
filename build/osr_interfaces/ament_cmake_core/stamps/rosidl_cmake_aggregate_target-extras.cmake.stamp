# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target osr_interfaces::osr_interfaces
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${osr_interfaces_TARGETS}.
if(osr_interfaces_TARGETS AND NOT TARGET osr_interfaces::osr_interfaces)
  add_library(osr_interfaces::osr_interfaces INTERFACE IMPORTED)
  set_target_properties(osr_interfaces::osr_interfaces PROPERTIES
    INTERFACE_LINK_LIBRARIES "${osr_interfaces_TARGETS}")
endif()
