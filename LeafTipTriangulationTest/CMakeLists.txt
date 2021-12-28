add_executable(LeafTipTriangulationTest)

message(STATUS "Creating target 'LeafTipTriangulationTest'")

if (MSVC)
    # warning level 4 and all warnings as errors
    target_compile_options(LeafTipTriangulationTest PRIVATE /W4 /WX)
else()
    # lots of warnings and all warnings as errors
    target_compile_options(LeafTipTriangulationTest PRIVATE -Wall -Wextra -pedantic -Werror)
endif()

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
