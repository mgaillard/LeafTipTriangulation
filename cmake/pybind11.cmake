if(TARGET pybind11)
  return()
endif()

message(STATUS "creating target 'pybind11'")

include(FetchContent)
FetchContent_Declare(
  pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG        v2.10.0
  GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(pybind11)
