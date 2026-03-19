# 1. 실행 파일 타겟 생성
add_executable(ysim main.cpp metal_impl.cpp)

# 2. OpenGL 찾기
find_package(OpenGL REQUIRED)

# 3. 로컬 lib 폴더 경로 지정 (glfw, glew가 최상단 lib 폴더에 있는 경우)
# CMAKE_SOURCE_DIR은 최상단 폴더(ysim)의 절대 경로를 의미합니다.
target_link_directories(ysim PRIVATE ${CMAKE_SOURCE_DIR}/lib)

# 4. ysim 실행 파일에 모든 의존성 연결
# ysim_core를 연결하는 순간, include 폴더 경로와 Eigen3 세팅이 ysim으로 자동 전파됩니다.
target_link_libraries(ysim PRIVATE 
    ysim_core     # 내 헤더 파일들과 Eigen3
    glfw          # lib 폴더 안에 있는 glfw
    glew          # lib 폴더 안에 있는 glew
    OpenGL::GL    # 시스템 OpenGL
)


configure_file(
    ${CMAKE_SOURCE_DIR}/src/shader/shader.vert
    ${CMAKE_BINARY_DIR}/shader.vert
    COPYONLY
)
configure_file(
    ${CMAKE_SOURCE_DIR}/src/shader/shader.geom
    ${CMAKE_BINARY_DIR}/shader.geom
    COPYONLY
)
configure_file(
    ${CMAKE_SOURCE_DIR}/src/shader/shader.frag
    ${CMAKE_BINARY_DIR}/shader.frag
    COPYONLY
)

