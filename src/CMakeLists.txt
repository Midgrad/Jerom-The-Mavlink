# Project
project(Jerom-The-Mavlink)

# Set output directory
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${MODULE_OUTPUT_DIRECTORY})

# Find Qt libraries
find_package(Qt5 ${QT_REQUIRED_VERSION} COMPONENTS Core REQUIRED)

# Qt plugin definition
add_definitions(-DQT_PLUGIN)

# Target
add_library(${PROJECT_NAME} MODULE "")

# Include
HEADER_DIRECTORIES(INCLUDES "./")
target_include_directories(${PROJECT_NAME} PRIVATE ${INCLUDES})

# Sources
file(GLOB_RECURSE SOURCES "*.h" "*.cpp")
target_sources(${PROJECT_NAME} PRIVATE ${SOURCES})

# Libraries
target_link_libraries(${PROJECT_NAME} PUBLIC kjarni loodsman mavlink Qt5::Core)

# Tests
if(TESTS_ENABLED)
#    add_subdirectory(tests)
endif(TESTS_ENABLED)
