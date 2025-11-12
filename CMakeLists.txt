cmake_minimum_required(VERSION 3.15)
project(dodeedum VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Glob source files
file(GLOB_RECURSE DODEEDUM_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

# Glob header files for IDE support
file(GLOB_RECURSE DODEEDUM_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h"
)

# Glob extension files
file(GLOB_RECURSE DODEEDUM_EXTENSION_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/extensions/*.cpp"
)

file(GLOB_RECURSE DODEEDUM_EXTENSION_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/extensions/*.h"
)

# Create static library
add_library(dodeedum STATIC
    ${DODEEDUM_SOURCES}
    ${DODEEDUM_HEADERS}
    ${DODEEDUM_EXTENSION_SOURCES}
    ${DODEEDUM_EXTENSION_HEADERS}
)

# Set include directories
target_include_directories(dodeedum
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/extensions
)

# Find and link GLM
find_package(glm QUIET)
if(glm_FOUND)
    target_link_libraries(dodeedum PUBLIC glm::glm)
else()
    # If GLM is not found, assume it's available in include path
    message(STATUS "GLM not found via find_package, assuming it's in include path")
endif()

# Set library properties
set_target_properties(dodeedum PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    PUBLIC_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/include/dodeedum.h;${CMAKE_CURRENT_SOURCE_DIR}/include/dodeedum_mesh.h"
)

# Optional: Install rules (uncomment if needed)
# install(TARGETS dodeedum
#     LIBRARY DESTINATION lib
#     ARCHIVE DESTINATION lib
#     PUBLIC_HEADER DESTINATION include/dodeedum
# )
