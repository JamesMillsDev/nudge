cmake_minimum_required(VERSION 3.20)
project(NudgeDev)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable testing
enable_testing()

# Add subdirectories
add_subdirectory(nudge)
add_subdirectory(tests)