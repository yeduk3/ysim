# Persistence-slice test executable. Backend-independent — does not require
# a Metal device or a GL context (docs/ARCHITECTURE.md §4.1, D-002).
add_executable(ysim_tests
    scene_io_test.cpp
)

target_link_libraries(ysim_tests PRIVATE ysim_core)
target_include_directories(ysim_tests PRIVATE ${CMAKE_SOURCE_DIR}/include)

enable_testing()
add_test(NAME ysim_tests COMMAND ysim_tests)
