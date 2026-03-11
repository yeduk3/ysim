//
//  camera.hpp
//  GPU_HW2
//
//  Created by 이용규 on 4/9/25.
//

#ifndef Camera_HPP
#define Camera_HPP

#ifndef PI
#define PI 3.14159265358979323846f
#endif


#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>

void cursorPosCallback(GLFWwindow *window, double xpos, double ypos);
void scrollCallback(GLFWwindow *window, double xoffset, double yoffset);

#if USE_GLM

#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/transform.hpp>

using namespace glm;

#else

#include "tinym.hpp"
using namespace tinym;

#endif
struct Camera {
    
    void setPosition(const vec3& initPos) {
        initPosition = initPos;
        curPosition = initPos;
        theta = 0;
        phi = 0;
        fovy = 45.f;
    }
    
    vec3 initPosition = vec3(0, 0, 10);
    vec3 look = vec3(0, 0, 0);
    vec3 up = vec3(0, 1, 0);
    
    float theta = 0;
    float phi = 0;
    float fovy = 45.f;
    
private:
    vec3 curPosition = vec3(0, 0, 10);
    
public:
    mat4 getRotate() {
        mat4 rotY = rotate(theta, vec3(0, 1, 0));
        mat4 rotX = rotate(phi, vec3(1, 0, 0));
        return rotY * rotX;
    }
    
    void rotatePosition() {
        curPosition = getRotate() * vec4(initPosition, 1);
    }
    
    mat4 lookAt() {
        return ::lookAt(curPosition, look, up);
    }
    
    mat4 perspective(float aspect, float zNear, float zFar) {
        return ::perspective(fovy * PI / 180.f, aspect, zNear, zFar);
    }
    
    void glfwSetCallbacks(GLFWwindow* window) {
        glfwSetCursorPosCallback(window, cursorPosCallback);
        glfwSetScrollCallback(window, scrollCallback);
    }
    
    vec3 getCurPosition() { return curPosition; }
};

Camera camera;

namespace comp {
inline float min(const float &a, const float &b) {
    return a > b ? b : a;
}
inline float max(const float &a, const float &b) {
    return a > b ? a : b;
}
inline float clamp(const float &value, const float &left, const float &right) {
    return max(left, min(value, right));
}
}

void cursorPosCallback(GLFWwindow *window, double xpos, double ypos)
{
    static double lastX = 0;
    static double lastY = 0;
    // when left mouse button clicked
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_1))
    {
        int w, h;
        glfwGetWindowSize(window, &w, &h);
        double dx = (xpos - lastX) / w;
        double dy = (ypos - lastY) / h;
//        if(glfwGetKey(window, GLFW_KEY_LEFT_CONTROL)) {
//            auto v = glm::normalize(camera.look - camera.getCurPosition());
//            auto r = glm::normalize(glm::cross(v, glm::vec3(0,1,0)));
//            auto u = glm::cross(r, v);
//            
//            auto t = glm::mat3(r, u, v) * glm::vec3(-dx, dy, 1);
//            camera.look += t;
//            camera.initPosition += t;
//        } else {
            camera.theta -= dx * PI; // related with y-axis rotation
            camera.phi -= dy * PI;   // related with x-axis rotation
            camera.phi = comp::clamp(camera.phi, -PI / 2 + 0.01f, PI / 2 - 0.01f);
            
            camera.rotatePosition();
//        }
    }
    // whenever, save current cursor position as previous one
    lastX = xpos;
    lastY = ypos;
}

void scrollCallback(GLFWwindow *window, double xoffset, double yoffset)
{
    // Y Offset - FOVY Modification
    camera.fovy -= yoffset / 10;
    camera.fovy = comp::clamp(camera.fovy, 0.01f, 180.f-0.01f);
}

#endif
