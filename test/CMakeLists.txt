# Persistence-slice test executable. Backend-independent — does not require
# a Metal device or a GL context (docs/ARCHITECTURE.md §4.1, D-002).
add_executable(ysim_tests
    scene_io_test.cpp
    sim_config_test.cpp
    motion_blend_test.cpp
)

target_link_libraries(ysim_tests PRIVATE ysim_core)
target_include_directories(ysim_tests PRIVATE ${CMAKE_SOURCE_DIR}/include)

# BDD-001 primitive geometry tests — separate executable because it owns its
# own DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN; doctest only allows one main() per
# binary.
add_executable(ysim_primitive_tests
    primitive_test.cpp
)
target_link_libraries(ysim_primitive_tests PRIVATE ysim_core)
target_include_directories(ysim_primitive_tests PRIVATE ${CMAKE_SOURCE_DIR}/include)

enable_testing()
add_test(NAME ysim_tests COMMAND ysim_tests)
add_test(NAME ysim_primitive_tests COMMAND ysim_primitive_tests)
