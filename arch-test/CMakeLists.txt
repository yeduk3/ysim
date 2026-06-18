# Standalone build for the arch-test engine. Independent of the root ysim
# target: reuses ../include (+ metal-cpp) and ../src/metal shaders, links only
# Metal/Foundation/QuartzCore + Eigen. No GL / ImGui / GLFW / Bullet / Assimp.
#
#   cmake -B build -S .            (from arch-test/)
#   cmake --build build
#   ./build/arch_runner
cmake_minimum_required(VERSION 3.10)
project(ysim_arch)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

get_filename_component(PARENT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)

find_package(Eigen3 5.0 REQUIRED NO_MODULE)

find_library(METAL_FRAMEWORK Metal)
find_library(FOUNDATION_FRAMEWORK Foundation)
find_library(QUARTZCORE_FRAMEWORK QuartzCore)

# --- metallib: reuse ../src/metal/*.metal unchanged, emit next to the exe ---
file(GLOB METAL_SOURCES "${PARENT_DIR}/src/metal/*.metal")
set(METAL_AIR_FILES "")
foreach(METAL_SOURCE ${METAL_SOURCES})
    get_filename_component(METAL_NAME ${METAL_SOURCE} NAME_WE)
    set(AIR_FILE ${CMAKE_CURRENT_BINARY_DIR}/${METAL_NAME}.air)
    add_custom_command(
        OUTPUT ${AIR_FILE}
        COMMAND xcrun -sdk macosx metal -c ${METAL_SOURCE} -o ${AIR_FILE}
        DEPENDS ${METAL_SOURCE}
        COMMENT "Compiling ${METAL_NAME}.metal -> ${METAL_NAME}.air")
    list(APPEND METAL_AIR_FILES ${AIR_FILE})
endforeach()
set(METAL_LIBRARY ${CMAKE_CURRENT_BINARY_DIR}/default.metallib)
add_custom_command(
    OUTPUT ${METAL_LIBRARY}
    COMMAND xcrun -sdk macosx metallib ${METAL_AIR_FILES} -o ${METAL_LIBRARY}
    DEPENDS ${METAL_AIR_FILES}
    COMMENT "Linking Metal library ${METAL_LIBRARY}")
add_custom_target(MetalKernels ALL DEPENDS ${METAL_LIBRARY})

file(GLOB_RECURSE ARCH_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_executable(arch_runner ${ARCH_SOURCES})
add_dependencies(arch_runner MetalKernels)

target_include_directories(arch_runner PRIVATE
    ${PARENT_DIR}/include
    ${PARENT_DIR}/include/metal-cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(arch_runner PRIVATE
    Eigen3::Eigen
    ${METAL_FRAMEWORK} ${FOUNDATION_FRAMEWORK} ${QUARTZCORE_FRAMEWORK})
target_compile_definitions(arch_runner PRIVATE YSIM_PROJECT_ROOT="${PARENT_DIR}")
