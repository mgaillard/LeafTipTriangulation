if(TARGET dlib::dlib)
    return()
endif()

message(STATUS "Third-party (external): creating target 'dlib::dlib'")

option(DLIB_GIF_SUPPORT OFF)
option(DLIB_JPEG_SUPPORT OFF)
option(DLIB_NO_GUI_SUPPORT OFF)
option(DLIB_PNG_SUPPORT OFF)
option(DLIB_USE_BLAS OFF)
option(DLIB_USE_CUDA OFF)
option(DLIB_USE_LAPACK OFF)
option(USE_SSE2_INSTRUCTIONS ON)
option(USE_SSE4_INSTRUCTIONS ON)
option(USE_AVX_INSTRUCTIONS ON)

include(FetchContent)
FetchContent_Declare(dlib
    GIT_REPOSITORY https://github.com/davisking/dlib.git
    GIT_TAG        v19.22
    GIT_SHALLOW    TRUE
)

FetchContent_GetProperties(dlib)
if(NOT dlib_POPULATED)
  FetchContent_Populate(dlib)
  add_subdirectory(${dlib_SOURCE_DIR}/dlib ${dlib_BINARY_DIR})
endif()

set_target_properties(dlib PROPERTIES FOLDER third_party)
