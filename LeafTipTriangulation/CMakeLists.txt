add_executable(LeafTipTriangulation)

message(STATUS "Creating target 'LeafTipTriangulation'")

set(HEADER_FILES 
    Camera.h
    ExportScene.h
    OBJWriter.h
    Ray.h
    RayMatching.h
    Reconstruction.h
    SyntheticData.h
    Triangulation.h
)

set(SRC_FILES
    Camera.cpp
    ExportScene.cpp
    main.cpp
    OBJWriter.cpp
    Ray.cpp
    RayMatching.cpp
    Reconstruction.cpp
    SyntheticData.cpp
    Triangulation.cpp
)

# Setup filters in Visual Studio
source_group("Header Files" FILES ${HEADER_FILES})
source_group("Source Files" FILES ${SRC_FILES})

# Copy the ReSharper config file
configure_file(
    "${CMAKE_SOURCE_DIR}/LeafTipTriangulation.sln.DotSettings"
    ${CMAKE_BINARY_DIR} COPYONLY
)

target_sources(LeafTipTriangulation
    PUBLIC
    ${HEADER_FILES}
    PRIVATE
    ${SRC_FILES}
)

target_include_directories(LeafTipTriangulation
    PUBLIC
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(LeafTipTriangulation 
    PRIVATE
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

if(TARGET spdlog::spdlog)
    add_custom_command(TARGET LeafTipTriangulation
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<TARGET_FILE:spdlog>" "$<TARGET_FILE_DIR:LeafTipTriangulation>"
    )
endif()
