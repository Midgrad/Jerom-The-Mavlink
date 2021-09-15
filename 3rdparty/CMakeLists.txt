# Project
project(mavlink)

# Interface library target
add_library(${PROJECT_NAME} INTERFACE)

# MAVLink version
if (MAVLINK_V2_ENABLE)
    set(MAVLINK_DIR "libmavlinkcpp_v2")
    message("Mavlink version 2 enabled")
else (MAVLINK_V2_ENABLE)
    set(MAVLINK_DIR "libmavlinkcpp_v1")
    message("Mavlink version 2 disabled")
endif (MAVLINK_V2_ENABLE)

target_include_directories(${PROJECT_NAME} INTERFACE "${MAVLINK_DIR}")

# Internal sources and headers
file(GLOB SOURCES "${MAVLINK_DIR}/*.h")
target_sources(${PROJECT_NAME} INTERFACE ${SOURCES})
