add_executable(LeafTipTriangulation)

message(STATUS "Creating target 'LeafTipTriangulation'")

set(HEADER_FILES 
    ExportScene.h
    OBJWriter.h
    Reconstruction.h
    SyntheticData.h
)

set(SRC_FILES
    ExportScene.cpp
    main.cpp
    OBJWriter.cpp
    Reconstruction.cpp
    SyntheticData.cpp
)

# Setup filters in Visual Studio
source_group("Header Files" FILES ${HEADER_FILES})
source_group("Source Files" FILES ${SRC_FILES})

if (MSVC)
    # warning level 4 and all warnings as errors
    target_compile_options(LeafTipTriangulation PRIVATE /W4 /WX)
else()
    # lots of warnings and all warnings as errors
    target_compile_options(LeafTipTriangulation PRIVATE -Wall -Wextra -pedantic -Werror)
endif()

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
    glm::glm
    OpenMP::OpenMP_CXX
    LeafTipTriangulationLib
)

if(TARGET spdlog::spdlog)
    add_custom_command(TARGET LeafTipTriangulation
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<TARGET_FILE:spdlog>" "$<TARGET_FILE_DIR:LeafTipTriangulation>"
    )
endif()
