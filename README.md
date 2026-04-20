# ysim

A GPU-accelerated physics simulation engine for cloth dynamics and rigid bodies on macOS. Rendering uses OpenGL, while simulation compute (physics, collision detection) runs on Metal.

## Features

- **Cloth simulation** with spring-based constraints (stretch, shear, bending)
  - Triangular mesh cloth
  - Fast grid-based cloth
- **Collision detection**
  - GPU spatial hashing broadphase with inline narrow phase
  - Linear BVH broadphase with point-triangle narrow phase
  - Self-collision support
- **Metal GPU compute** for physics integration, force computation, BVH construction, and collision detection
- **Real-time GUI** via Dear ImGui with mesh inspector, profiler, and simulation controls
- **OBJ mesh loading** with material support

## Requirements

- macOS with Metal support
- CMake 3.10+
- C++17 compiler
- Xcode Command Line Tools (for `xcrun metal` shader compilation)
- [Eigen 5.0+](https://eigen.tuxfamily.org/)
- GLFW 3.4 and GLEW 2.2 (bundled in `lib/`)
- OpenGL

## Build

```bash
cmake -B build
cmake --build build
./build/src/ysim
```

### Xcode

```bash
mkdir buildxc && cd buildxc
cmake -G Xcode ..
```

Open the generated `ysim.xcodeproj` in Xcode.

## Project Structure

```
src/
  main.cpp              Core application: simulation loop, GUI, scene setup
  metal/
    physics.metal       Force computation, spring forces, cloth grid physics
    bvh.metal           Linear BVH construction, Morton codes, radix sort, broadphase query
    spatialhashing.metal  GPU spatial hashing broadphase with inline narrow phase
    bruteforce.metal    Point-triangle narrow phase collision
    common_types.metalh Shared data structures (BroadCollision, NarrowCollision)
  shader/
    shader.vert/geom/frag   Phong lighting with wireframe overlay
    line.vert/frag          Debug line rendering
include/
  camera.hpp            Camera with mouse/scroll interaction
  YGLWindow.hpp         GLFW/OpenGL window wrapper
  program.hpp           OpenGL shader program management
  tinym.hpp             Lightweight math library (vec3, vec4, mat4)
  objreader.hpp         OBJ/MTL file loader
  FrameProfiler.hpp     Per-frame timing profiler
  metal-cpp/            C++ wrapper for Metal API
third_party/
  imgui/                Dear ImGui (GLFW + OpenGL3 backend)
assets/                 OBJ mesh files (teapot, human, horse, camel)
profiles/               CSV profiling export directory
```

## Architecture

The engine uses a dual-GPU strategy: **OpenGL** for rendering and **Metal compute** for simulation. Core types are template-parameterized on backend (`CPU`/`CUDA`/`METAL`) and precision, allowing the same data structures to work across backends.

### Collision Pipeline

Two broadphase implementations are available:

- **Spatial Hashing** -- Uniform grid hashing with 8-pass cell-type collision detection. Produces narrow phase results directly (inline point-triangle distance checks on GPU).
- **Linear BVH** -- Morton code-based BVH with radix sort, bottom-up AABB construction, and tree traversal queries. Feeds into a separate narrow phase pass.

### Metal Shader Build

`.metal` source files are compiled to `.air` intermediates via `xcrun metal`, then linked into `default.metallib` via `xcrun metallib`. This is handled by custom CMake commands.

## License

This project is for personal/research use.
