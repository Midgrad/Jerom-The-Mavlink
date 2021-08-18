# Project
project(mavlink)

# Interface library target
add_library(${PROJECT_NAME} INTERFACE)

set(MAVLINK_V2 TRUE)

# MAVLink version
if (MAVLINK_V2)
    set(MAVLINK_DIR "libmavlinkcpp_v2")
    message(MAVLINK_V2)
else (MAVLINK_V2)
    set(MAVLINK_DIR "libmavlinkcpp_v1")
    message(MAVLINK_V2)
endif (MAVLINK_V2)

target_include_directories(${PROJECT_NAME} INTERFACE "${MAVLINK_DIR}")

# Internal sources and headers
file(GLOB_RECURSE SOURCES "${MAVLINK_DIR}/*.h")
target_sources(${PROJECT_NAME} INTERFACE ${SOURCES})
