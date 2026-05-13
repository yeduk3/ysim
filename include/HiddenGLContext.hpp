#ifndef YSIM_HIDDEN_GL_CONTEXT_HPP
#define YSIM_HIDDEN_GL_CONTEXT_HPP

// D-032: offscreen GL context for the FBO render harness. Parallel
// symbol to `YGLWindow` (include/YGLWindow.hpp) — does NOT modify the
// production class. Difference: `GLFW_VISIBLE = GLFW_FALSE` before
// window creation so the macOS Window Server doesn't show the
// window. The GL context is otherwise identical (Apple Silicon GL 4.1
// core; same hints as YGLWindow).
//
// Used by Block 25 and Block 27 in `runSelfTest` to render two PBR
// passes at different roughness values and verify the shader respects
// the uniform / to confirm the loader's failure semantics. On hosts
// where GLFW or GLEW init fails (no display server, no GL driver,
// etc.), `ok` stays false and the using block SKIPs rather than FAILs
// — same SKIP-vs-FAIL discipline as the Metal-less host branch at the
// top of `runSelfTest`.
//
// D-034: symmetric construction/destruction. Each constructor step
// that succeeded is undone in the destructor. The `glfwInitialized`
// flag tracks whether `glfwInit()` returned truthy; the destructor
// consults it to decide whether to call `glfwTerminate()`. Previously
// the destructor deliberately skipped `glfwTerminate()` to avoid
// invalidating a future `YGLWindow` in the same process — but
// `--self-test` mode and default-main mode are mutually exclusive at
// runtime (the binary either runs the self-test then exits, or runs
// the GUI; it does not transition between the two), so the cleanup is
// a correctness win, not a regression risk.
//
// **Documented limitation:** GLFW's `glfwInit()` is NOT ref-counted;
// `glfwTerminate()` undoes ALL initialization regardless of how many
// times `glfwInit()` was called. If two `HiddenGLContext` instances
// are created concurrently (overlapping lifetimes), the first
// destructor's `glfwTerminate()` will invalidate the second context's
// window. Current usage (Block 25 then Block 27, sequential
// non-overlapping) is safe; future concurrent use would need a
// process-global init counter instead of the per-instance flag.

#include <GL/glew.h>
#include <GLFW/glfw3.h>

struct HiddenGLContext {
    GLFWwindow* window = nullptr;
    bool ok = false;
    // D-034: tracks whether `glfwInit()` succeeded inside this
    // constructor. The destructor calls `glfwTerminate()` only if
    // this flag is true — otherwise an early-init failure path
    // (glfwInit itself returned false) would call glfwTerminate on
    // an uninitialized library, which is UB per GLFW docs.
    bool glfwInitialized = false;

    HiddenGLContext(int width, int height) {
        if (!glfwInit()) return;
        glfwInitialized = true;
#ifdef __APPLE__
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#endif
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        window = glfwCreateWindow(width, height, "ysim-hidden", nullptr, nullptr);
        if (!window) return;
        glfwMakeContextCurrent(window);
        if (glewInit() != GLEW_OK) {
            glfwDestroyWindow(window);
            window = nullptr;
            return;
        }
        ok = true;
    }

    // D-034: symmetric cleanup. Destroy the window if one was created,
    // then terminate GLFW if this instance initialized it. The order
    // matters — glfwDestroyWindow requires a live GLFW state.
    ~HiddenGLContext() {
        if (window) glfwDestroyWindow(window);
        if (glfwInitialized) glfwTerminate();
    }

    HiddenGLContext(const HiddenGLContext&) = delete;
    HiddenGLContext& operator=(const HiddenGLContext&) = delete;
};

#endif  // YSIM_HIDDEN_GL_CONTEXT_HPP
