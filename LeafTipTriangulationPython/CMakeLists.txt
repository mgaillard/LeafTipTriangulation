pybind11_add_module(LeafTipTriangulationPython main.cpp)

message(STATUS "Creating target 'LeafTipTriangulationPython'")

if (MSVC)
    # warning level 4 and all warnings as errors
    target_compile_options(LeafTipTriangulationPython PRIVATE /W4 /WX)
else()
    # lots of warnings and all warnings as errors
    target_compile_options(LeafTipTriangulationPython PRIVATE -Wall -Wextra -pedantic -Werror)
endif()

target_link_libraries(LeafTipTriangulationPython
    PRIVATE
    LeafTipTriangulationLib
)

