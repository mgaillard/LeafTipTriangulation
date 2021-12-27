# Source: https://github.com/adobe/lagrange/blob/main/cmake/recipes/external/spdlog.cmake

if(TARGET spdlog::spdlog)
    return()
endif()

message(STATUS "Third-party (external): creating target 'spdlog::spdlog'")

include(FetchContent)
FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG        v1.9.2
    GIT_SHALLOW    TRUE
)

option(SPDLOG_INSTALL "Generate the install target" ON)
set(CMAKE_INSTALL_DEFAULT_COMPONENT_NAME "spdlog")
FetchContent_MakeAvailable(spdlog)

set_target_properties(spdlog PROPERTIES POSITION_INDEPENDENT_CODE ON)
set_target_properties(spdlog PROPERTIES FOLDER third_party)
