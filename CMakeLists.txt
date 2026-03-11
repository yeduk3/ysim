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
# 이 과정을 통해 physics.metal이 컴파일되어 실행 파일 옆에 배치됩니다.
set(METAL_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/src/physics.metal)
set(METAL_LIBRARY ${CMAKE_CURRENT_BINARY_DIR}/physics.metallib)

add_custom_command(
    OUTPUT ${METAL_LIBRARY}
    COMMAND xcrun -sdk macosx metal -c ${METAL_SOURCE} -o ${CMAKE_CURRENT_BINARY_DIR}/physics.air
    COMMAND xcrun -sdk macosx metallib ${CMAKE_CURRENT_BINARY_DIR}/physics.air -o ${METAL_LIBRARY}
    DEPENDS ${METAL_SOURCE}
    COMMENT "Compiling Metal Compute Kernel..."
)

# 메인 타겟이 Metal 라이브러리 컴파일에 의존하도록 설정
add_custom_target(MetalKernels ALL DEPENDS ${METAL_LIBRARY})

# 4. src 폴더의 CMakeLists.txt 실행
add_subdirectory(src)

add_dependencies(ysim MetalKernels)

