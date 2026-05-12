#ifndef YSIM_HIDDEN_GL_CONTEXT_HPP
#define YSIM_HIDDEN_GL_CONTEXT_HPP

// D-032: offscreen GL context for the FBO render harness. Parallel
// symbol to `YGLWindow` (include/YGLWindow.hpp) — does NOT modify the
// production class. Difference: `GLFW_VISIBLE = GLFW_FALSE` before
// window creation so the macOS Window Server doesn't show the
// window. The GL context is otherwise identical (Apple Silicon GL 4.1
// core; same hints as YGLWindow).
//
// Used by Block 25 in `runSelfTest` to render two PBR passes at
// different roughness values and verify the shader respects the
// uniform. On hosts where GLFW or GLEW init fails (no display
// server, no GL driver, etc.), `ok` stays false and Block 25 SKIPs
// rather than FAILs — same SKIP-vs-FAIL discipline as the Metal-less
// host branch at the top of `runSelfTest`.
//
// Destructor destroys the window but deliberately does NOT call
// `glfwTerminate()` — if the production `YGLWindow` is still alive
// in the same process (it isn't during `--self-test`, but might be in
// future test embeddings), terminating GLFW would invalidate its
// window. Process-exit cleanup is implicit and harmless.

#include <GL/glew.h>
#include <GLFW/glfw3.h>

struct HiddenGLContext {
    GLFWwindow* window = nullptr;
    bool ok = false;

    HiddenGLContext(int width, int height) {
        if (!glfwInit()) return;
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

    ~HiddenGLContext() {
        if (window) glfwDestroyWindow(window);
    }

    HiddenGLContext(const HiddenGLContext&) = delete;
    HiddenGLContext& operator=(const HiddenGLContext&) = delete;
};

#endif  // YSIM_HIDDEN_GL_CONTEXT_HPP
