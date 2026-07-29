cmake_minimum_required(VERSION 3.10)
project(arch_test_handmade CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(arch_test_handmade main.cpp)
# src/ for the engine headers; /opt/homebrew/include for nlohmann/json (matches
# the real ysim, which also uses nlohmann for its RunConfig JSON).
target_include_directories(arch_test_handmade PRIVATE src /opt/homebrew/include)
# Registry JSON scenes are listed with source-relative paths; this lets the
# binary resolve them from any cwd (same trick as ysim's YSIM_PROJECT_ROOT).
target_compile_definitions(arch_test_handmade
    PRIVATE ATH_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
