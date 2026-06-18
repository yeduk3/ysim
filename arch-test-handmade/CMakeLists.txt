cmake_minimum_required(VERSION 3.10)
project(arch_test_handmade CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(arch_test_handmade main.cpp)
target_include_directories(arch_test_handmade PRIVATE src)
