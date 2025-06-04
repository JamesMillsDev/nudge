# Recursively glob all source files
file(GLOB_RECURSE NUDGE_SOURCES
    "src/*.cpp"
    "src/*.c"
    "src/*.cxx"
    "src/*.cc"
)

# Recursively glob all header files
file(GLOB_RECURSE NUDGE_HEADERS
    "include/*.h"
    "include/*.hpp"
    "include/*.hxx"
    "src/*.h"
    "src/*.hpp"
    "src/*.hxx"
)

# Create the static library
add_library(nudge STATIC
    ${NUDGE_SOURCES}
    ${NUDGE_HEADERS}
)

# Set include directories for the library
target_include_directories(nudge PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Optional: Set target properties
set_target_properties(nudge PROPERTIES
    OUTPUT_NAME "nudge"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

# Optional: Print found files for debugging
message(STATUS "Nudge source files found:")
foreach(source ${NUDGE_SOURCES})
    message(STATUS "  ${source}")
endforeach()

message(STATUS "Nudge header files found:")
foreach(header ${NUDGE_HEADERS})
    message(STATUS "  ${header}")
endforeach()