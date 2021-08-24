# Project
project(Jerom-The-Mavlink)

# Copy resources
copy_post_cmake("route_type_mavlink.json" ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/route_types)

# Mavlink 3rdparty
add_subdirectory(3rdparty)

# Find Qt libraries
find_package(Qt5 ${QT_REQUIRED_VERSION} COMPONENTS Core REQUIRED)

# Qt plugin definition
add_definitions(-DQT_PLUGIN)

# Target
add_library(${PROJECT_NAME} MODULE "")

# Include
target_include_directories(${PROJECT_NAME} PUBLIC "src")

# Sources
file(GLOB SOURCES "src/*")
target_sources(${PROJECT_NAME} PRIVATE ${SOURCES})

# Libraries
target_link_libraries(${PROJECT_NAME} PUBLIC kjarni Qt5::Core loodsman mavlink)

copy_post_cmake("link_config.json" ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
