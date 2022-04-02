add_library(LeafTipTriangulationLib STATIC)

message(STATUS "Creating target 'LeafTipTriangulationLib'")

set(HEADER_FILES
    include/utils/warnoff.h
    include/utils/warnon.h
    include/Camera.h
    include/ConvertUtils.h
    include/Ray.h
    include/RayMatching.h
    include/Triangulation.h
)

set(SRC_FILES
    src/Camera.cpp
    src/ConvertUtils.cpp
    src/Ray.cpp
    src/RayMatching.cpp
    src/Triangulation.cpp
)

# Setup filters in Visual Studio
source_group("Header Files" FILES ${HEADER_FILES})
source_group("Source Files" FILES ${SRC_FILES})

if (MSVC)
    # warning level 4 and all warnings as errors
    target_compile_options(LeafTipTriangulationLib PRIVATE /W4 /WX)
else()
    # lots of warnings and all warnings as errors
    target_compile_options(LeafTipTriangulationLib PRIVATE -Wall -Wextra -pedantic -Werror)
endif()

target_sources(LeafTipTriangulationLib
    PUBLIC
    ${HEADER_FILES}
    PRIVATE
    ${SRC_FILES}
)

target_include_directories(LeafTipTriangulationLib
    PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(LeafTipTriangulationLib 
    PUBLIC
    spdlog::spdlog
    dlib::dlib
    glm::glm
    opencv_core
    opencv_imgcodecs
    opencv_imgproc
    opencv_highgui
    opencv_calib3d
    OpenMP::OpenMP_CXX
)
