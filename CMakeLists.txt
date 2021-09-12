# Copy resources
copy_post_cmake("route_type_mavlink.json" ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/route_types)
copy_post_cmake("link_config.json" ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# Mavlink 3rdparty
add_subdirectory(3rdparty)

# Mavlink module
add_subdirectory(impl)
add_subdirectory(module)

