if(TARGET Catch2::Catch2)
    return()
endif()

message(STATUS "Third-party (external): creating target 'Catch2::Catch2'")

include(FetchContent)
FetchContent_Declare(
    catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v2.13.7
    GIT_SHALLOW TRUE
)

FetchContent_GetProperties(catch2)
if(NOT catch2_POPULATED)
  FetchContent_Populate(catch2)
  # Include Catch2
  add_subdirectory(${catch2_SOURCE_DIR} ${catch2_BINARY_DIR})
  # Make the function `catch_discover_tests` available to register tests.
  include("${catch2_SOURCE_DIR}/contrib/Catch.cmake")
endif()
