cmake_minimum_required(VERSION 3.10)
project(ysim)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. 헤더 온리 라이브러리(인터페이스) 타겟 생성
add_library(ysim_core INTERFACE)

# 2. include 폴더를 ysim_core의 기본 경로로 지정
# 이렇게 하면 include 안의 헤더들끼리도 서로 경로 없이 #include "YMath.hpp" 식으로 부를 수 있습니다.
target_include_directories(ysim_core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# 3. 공통 의존성 (Eigen) 찾기 및 묶기
# 헤더 파일에서 Eigen을 사용하므로 여기서 연결해 줍니다.
find_package(Eigen3 5.0 REQUIRED NO_MODULE)
target_link_libraries(ysim_core INTERFACE Eigen3::Eigen)

# --- 1. Apple Metal 프레임워크 링킹 ---
if (APPLE)
    find_library(METAL_FRAMEWORK Metal)
    find_library(FOUNDATION_FRAMEWORK Foundation)
    find_library(QUARTZCORE_FRAMEWORK QuartzCore)
    
    target_link_libraries(ysim_core INTERFACE 
        ${METAL_FRAMEWORK} 
        ${FOUNDATION_FRAMEWORK} 
        ${QUARTZCORE_FRAMEWORK}
    )
    target_include_directories(ysim_core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include/metal-cpp)
endif()


# --- 2. .metal 소스 코드를 .metallib 로 컴파일하는 커스텀 규칙 ---
file(GLOB METAL_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/metal/*.metal")

set(METAL_AIR_FILES "")
foreach(METAL_SOURCE ${METAL_SOURCES})
    get_filename_component(METAL_NAME ${METAL_SOURCE} NAME_WE)
    set(AIR_FILE ${CMAKE_CURRENT_BINARY_DIR}/${METAL_NAME}.air)

    add_custom_command(
        OUTPUT ${AIR_FILE}
        COMMAND xcrun -sdk macosx metal -c ${METAL_SOURCE} -o ${AIR_FILE}
        DEPENDS ${METAL_SOURCE}
        COMMENT "Compiling ${METAL_NAME}.metal -> ${METAL_NAME}.air"
    )

    list(APPEND METAL_AIR_FILES ${AIR_FILE})
endforeach()

set(METAL_LIBRARY ${CMAKE_CURRENT_BINARY_DIR}/default.metallib)

add_custom_command(
    OUTPUT ${METAL_LIBRARY}
    COMMAND xcrun -sdk macosx metallib ${METAL_AIR_FILES} -o ${METAL_LIBRARY}
    DEPENDS ${METAL_AIR_FILES}
    COMMENT "Linking Metal library ${METAL_LIBRARY}"
)

add_custom_target(MetalKernels ALL DEPENDS ${METAL_LIBRARY})

# root set
set(YSIM_PROJECT_ROOT_DIR "${PROJECT_SOURCE_DIR}")

# Bullet 3.25 — D-040. Vendored under third_party/bullet3 as a git submodule.
# Disable demos/extras/tests/Bullet3-experimental/shared/double-precision/
# pybullet so the sub-project only builds the three modules ysim links:
# BulletDynamics + BulletCollision + LinearMath.
# Bullet 3.25's CMakeLists declares cmake_minimum_required(VERSION 2.x);
# modern CMake (4.x) needs explicit policy compat to accept that.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
set(BUILD_BULLET2_DEMOS    OFF CACHE BOOL "" FORCE)
set(BUILD_CPU_DEMOS        OFF CACHE BOOL "" FORCE)
set(BUILD_EXTRAS           OFF CACHE BOOL "" FORCE)
set(BUILD_OPENGL3_DEMOS    OFF CACHE BOOL "" FORCE)
set(BUILD_UNIT_TESTS       OFF CACHE BOOL "" FORCE)
set(INSTALL_LIBS           OFF CACHE BOOL "" FORCE)
set(USE_GRAPHICAL_BENCHMARK OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS      OFF CACHE BOOL "" FORCE)
set(BUILD_BULLET3          OFF CACHE BOOL "" FORCE)
set(USE_DOUBLE_PRECISION   OFF CACHE BOOL "" FORCE)
set(BUILD_PYBULLET         OFF CACHE BOOL "" FORCE)
add_subdirectory(third_party/bullet3 EXCLUDE_FROM_ALL)

# 4. src 폴더의 CMakeLists.txt 실행
add_subdirectory(src)

add_dependencies(ysim MetalKernels)

# 5. 테스트 (persistence slice — D-002)
enable_testing()
add_subdirectory(test)

