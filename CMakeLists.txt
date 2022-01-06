# Project
project(Jerom-The-Mavlink)

# MAVLink version option
option(MAVLINK_V2_ENABLE "MAVLink version 2 includes" ON)
if (MAVLINK_V2_ENABLE)
    add_definitions(-DMAVLINK_V2)
endif (MAVLINK_V2_ENABLE)

# Mavlink 3rdparty
add_subdirectory(3rdparty)

# Set output directory
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${MODULE_OUTPUT_DIRECTORY})

# Find Qt libraries
find_package(Qt5 ${QT_REQUIRED_VERSION} COMPONENTS Core REQUIRED)

# Copy resources
copy_post_cmake("link_config.json" ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# Qt plugin definition
add_definitions(-DQT_PLUGIN)

# Target
add_library(${PROJECT_NAME} MODULE "")

# Include
HEADER_DIRECTORIES(INCLUDES "src")
target_include_directories(${PROJECT_NAME} PRIVATE ${INCLUDES})

# Sources
file(GLOB_RECURSE SOURCES "src/*.h" "src/*.cpp")
target_sources(${PROJECT_NAME} PRIVATE ${SOURCES})

# Local resources
file(GLOB LOC_QRC_FILES "qml/*.qrc" "resources/*.qrc")

# Compile resources
qt5_add_resources(QRC_SOURCES ${QRC_FILES} ${LOC_QRC_FILES})
target_sources(${PROJECT_NAME} PRIVATE ${QRC_SOURCES})

# Libraries
target_link_libraries(${PROJECT_NAME} PUBLIC kjarni loodsman mavlink Qt5::Core)

# Tests
if(TESTS_ENABLED)
#    add_subdirectory(tests)
endif(TESTS_ENABLED)


