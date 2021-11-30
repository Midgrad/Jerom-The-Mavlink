# Copy resources
copy_post_cmake("link_config.json" ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# MAVLink version option
option(MAVLINK_V2_ENABLE "MAVLink version 2 includes" ON)
if (MAVLINK_V2_ENABLE)
    add_definitions(-DMAVLINK_V2)
endif (MAVLINK_V2_ENABLE)

# Mavlink 3rdparty
add_subdirectory(3rdparty)

# Mavlink module
add_subdirectory(src)


