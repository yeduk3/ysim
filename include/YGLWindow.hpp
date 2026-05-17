//
//  YGLWindow.hpp
//  GPUPractice
//
//  Created by 이용규 on 6/11/25.
//

#ifndef YGLWindow_HPP
#define YGLWindow_HPP

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <OpenGL/gl.h>


#include <framebuffer.hpp>
#include <program.hpp>
 


struct YGLWindow;


struct YGLWindowPool {
    static YGLWindowPool& get() {
        static YGLWindowPool p;
        return p;
    }
    
    void registerWindow(YGLWindow* yw);
    YGLWindow* search(GLFWwindow* window);
    
    YGLWindowPool(const YGLWindowPool&) = delete;
    YGLWindowPool& operator=(const YGLWindowPool&) = delete;
    
private:
    YGLWindowPool() = default;
    ~YGLWindowPool() = default;
    
    std::vector<YGLWindow*> windows_;
};





struct YGLWindow {
    typedef std::function<void()> YGLFunc;
    
    YGLWindow(const int width, const int height, const char* windowName) {
        // GLFW Init & Setting
        glfwInit();
        
#ifdef __APPLE__
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#endif
        
        window_ = glfwCreateWindow(width, height, windowName, 0, 0);
        glfwMakeContextCurrent(window_);
        
        // GLEW Init
        glewInit();
        
        // Some Settings
        glfwSwapInterval(1);
        
        YGLWindowPool::get().registerWindow(this);
        
        
        glfwSetFramebufferSizeCallback(window_, framebufferResizeCallback);
        
        int w, h;
        glfwGetFramebufferSize(window_, &w, &h);
        framebufferResize(w, h);
        
        std::cout<<"YGLWindow created"<<std::endl;
    }
    ~YGLWindow() {
        glfwDestroyWindow(window_);
        glfwTerminate();
    }
    
    
    void initFunc  (YGLFunc init) { init_ = init; }
    void renderFunc(YGLFunc render) { render_ = render; }
    
    void mainLoop() {
        init_();
        
        while(!glfwWindowShouldClose(window_) /* && !glfwGetKey(window_, GLFW_KEY_ESCAPE) */) {
            render_();
            
            glfwSwapBuffers(window_);
            glfwPollEvents();
        }
    }
    void mainLoop(YGLFunc init, YGLFunc render) {
        initFunc(init);
        renderFunc(render);
        mainLoop();
    }
    void mainLoop(YGLFunc render) {
        renderFunc(render);
        mainLoop();
    }
    
    
    
    
    GLFWwindow* getGLFWWindow() { return window_; }
    
    void framebufferResize(const int width, const int height) {
        width_  = width;
        height_ = height;
        glViewport(0, 0, width_, height_);
    }
    
    static void framebufferResizeCallback(GLFWwindow* window, int width, int height) {
        auto* win = YGLWindowPool::get().search(window);
        if(win) win->framebufferResize(width, height);
        else cerr<<"Error on framebufferResizeCallback's window pointer"<<endl;
    }
    
    int   width()  const { return width_; }
    int   height() const { return height_; }
    float aspect() const { return width_/(float)height_; }
    
private:
    GLFWwindow* window_ = nullptr;
    int width_, height_;
    
    const YGLFunc defRenderFunc = [](){};
    YGLFunc render_ = defRenderFunc;
    YGLFunc init_   = defRenderFunc;
};






inline void YGLWindowPool::registerWindow(YGLWindow* yw) {
    windows_.push_back(yw);
}

inline YGLWindow* YGLWindowPool::search(GLFWwindow* window) {
    for (auto *yw : windows_) {
        if (yw->getGLFWWindow() == window) return yw;
    }
    return nullptr;
}

#endif
