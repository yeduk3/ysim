# 1. 실행 파일 타겟 생성
add_executable(ysim
    main.cpp
    metal_impl.cpp
    profiler_gui.cpp
    mesh_inspector_gui.cpp
    BulletRigidPhysicsBackend.cpp
    ${CMAKE_SOURCE_DIR}/third_party/imgui/imgui.cpp
    ${CMAKE_SOURCE_DIR}/third_party/imgui/imgui_draw.cpp
    ${CMAKE_SOURCE_DIR}/third_party/imgui/imgui_tables.cpp
    ${CMAKE_SOURCE_DIR}/third_party/imgui/imgui_widgets.cpp
    ${CMAKE_SOURCE_DIR}/third_party/imgui/backends/imgui_impl_glfw.cpp
    ${CMAKE_SOURCE_DIR}/third_party/imgui/backends/imgui_impl_opengl3.cpp
)

# 2. OpenGL 찾기
find_package(OpenGL REQUIRED)
find_path(GLFW_NATIVE_INCLUDE_DIR GLFW/glfw3native.h
    PATHS
        /opt/homebrew/include
        /opt/homebrew/Cellar/glfw/3.4/include
        /usr/local/include
)

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
    # D-040: Bullet 3.25 rigid physics backend.
    BulletDynamics
    BulletCollision
    LinearMath
)

target_include_directories(ysim PRIVATE
    ${CMAKE_SOURCE_DIR}/third_party/imgui
    ${CMAKE_SOURCE_DIR}/third_party/imgui/backends
    ${CMAKE_SOURCE_DIR}/third_party/bullet3/src
)

if (GLFW_NATIVE_INCLUDE_DIR)
    target_include_directories(ysim PRIVATE ${GLFW_NATIVE_INCLUDE_DIR})
endif()

target_compile_definitions(ysim PRIVATE
    IMGUI_IMPL_OPENGL_LOADER_GLEW
)

target_compile_definitions(ysim PRIVATE
    YSIM_PROJECT_ROOT="${YSIM_PROJECT_ROOT_DIR}"
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

configure_file(
    ${CMAKE_SOURCE_DIR}/src/shader/line.vert
    ${CMAKE_BINARY_DIR}/line.vert
    COPYONLY
)
configure_file(
    ${CMAKE_SOURCE_DIR}/src/shader/line.frag
    ${CMAKE_BINARY_DIR}/line.frag
    COPYONLY
)
configure_file(
    ${CMAKE_SOURCE_DIR}/src/shader/id.vert
    ${CMAKE_BINARY_DIR}/id.vert
    COPYONLY
)
configure_file(
    ${CMAKE_SOURCE_DIR}/src/shader/id.frag
    ${CMAKE_BINARY_DIR}/id.frag
    COPYONLY
)
configure_file(
    ${CMAKE_SOURCE_DIR}/src/shader/idpoint.vert
    ${CMAKE_BINARY_DIR}/idpoint.vert
    COPYONLY
)
configure_file(
    ${CMAKE_SOURCE_DIR}/src/shader/idpoint.frag
    ${CMAKE_BINARY_DIR}/idpoint.frag
    COPYONLY
)
configure_file(
    ${CMAKE_SOURCE_DIR}/src/shader/point.vert
    ${CMAKE_BINARY_DIR}/point.vert
    COPYONLY
)
configure_file(
    ${CMAKE_SOURCE_DIR}/src/shader/point.frag
    ${CMAKE_BINARY_DIR}/point.frag
    COPYONLY
)
add_custom_command(TARGET ysim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/assets
            $<TARGET_FILE_DIR:ysim>/assets
)

