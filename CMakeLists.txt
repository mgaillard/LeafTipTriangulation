cmake_minimum_required(VERSION 3.16.3)

project(LeafTipTriangulationProject VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Activate OpenMP
find_package(OpenMP REQUIRED)

# Add CMake recipes
list(PREPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

# Dependencies for application
include(spdlog)
include(glm)
include(dlib)
include(opencv)
include(pybind11)

# Dependencies for tests
include(catch2)
include(CTest)

# Lib Python is required for the Python module
find_package(Python COMPONENTS Interpreter Development REQUIRED)

add_subdirectory(LeafTipTriangulationLib)
add_subdirectory(LeafTipTriangulation)
add_subdirectory(LeafTipTriangulationTest)
add_subdirectory(LeafTipTriangulationPython)

# Set the project as startup project in Visual Studio
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT LeafTipTriangulation)

# Copy the ReSharper config file
configure_file(
    "${CMAKE_SOURCE_DIR}/LeafTipTriangulationProject.sln.DotSettings"
    ${CMAKE_BINARY_DIR} COPYONLY
)
