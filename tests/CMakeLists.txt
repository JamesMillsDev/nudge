# Include FetchContent module
include(FetchContent)

# Fetch Google Test
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/releases/download/v1.17.0/googletest-1.17.0.tar.gz
)

# Prevent GTest/GMock from installing
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
set(INSTALL_GMOCK OFF CACHE BOOL "" FORCE)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(googletest)

# Additional exclusion (belt and suspenders approach)
set_target_properties(gtest PROPERTIES EXCLUDE_FROM_ALL TRUE)
set_target_properties(gtest_main PROPERTIES EXCLUDE_FROM_ALL TRUE)
set_target_properties(gmock PROPERTIES EXCLUDE_FROM_ALL TRUE)
set_target_properties(gmock_main PROPERTIES EXCLUDE_FROM_ALL TRUE)

# Create test executable
add_executable(NudgeTests
    Vector2Tests.cpp
    Matrix2Tests.cpp
    Vector3Tests.cpp
    Matrix3Tests.cpp
    Vector4Tests.cpp
    Matrix4Tests.cpp
    MathFTests.cpp
    QuaternionTests.cpp
)

# Link libraries
target_link_libraries(NudgeTests
    PRIVATE
    nudge
    gtest_main
    gtest
)

# Set the working directory for the tests (helps with VS debugging)
set_target_properties(NudgeTests PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)

# Include Google Test CMake functions
include(GoogleTest)

# Discover tests automatically - this is what makes VS Test Explorer work
gtest_discover_tests(NudgeTests
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)