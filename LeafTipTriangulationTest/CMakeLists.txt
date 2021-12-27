add_executable(LeafTipTriangulationTest)

message(STATUS "Creating target 'LeafTipTriangulationTest'")

target_sources(LeafTipTriangulationTest
    PRIVATE
    main.cpp
    test.cpp
)

target_link_libraries(LeafTipTriangulationTest
    PRIVATE
    LeafTipTriangulationLib
    Catch2::Catch2
)

# Activate profiling for the RelWithDebInfo target on Windows
if (MSVC) 
  target_link_options(LeafTipTriangulationTest PRIVATE $<$<CONFIG:RELWITHDEBINFO>:/PROFILE>)
endif()

catch_discover_tests(LeafTipTriangulationTest WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
