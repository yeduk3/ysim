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
#include <cmath>
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


inline float toRadian(float angle) { return angle*PI/180.f; }

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
    
    float zNear=0.1, zFar=1000.0;
    float aspect = 0;

private:
    vec3 curPosition = vec3(0, 0, 10);
    
public:
    mat4 getRotate() {
        mat4 rotY = rotate(theta, vec3(0, 1, 0));
        mat4 rotX = rotate(phi, vec3(1, 0, 0));
        return rotY * rotX;
    }
    
    void rotatePosition() {
        // Orbit rotates initPosition about the world origin; `look` is
        // the pan pivot, added after rotation so the camera and its
        // target translate together (look defaults to origin, so this
        // is a no-op until pan() moves it — orbit behavior unchanged).
        vec4 rp = getRotate() * vec4(initPosition, 1);
        curPosition = vec3(rp.x + look.x, rp.y + look.y, rp.z + look.z);
    }

    // Pan the view by a normalized cursor delta (fraction of the
    // window). Shifts the look pivot along the camera's right/up axes,
    // scaled by the orbit distance so the framing feels consistent at
    // any zoom. rotatePosition() then carries curPosition along.
    void pan(float ndx, float ndy) {
        vec3 cp = curPosition;
        vec3 fwd = vec3(look.x - cp.x, look.y - cp.y, look.z - cp.z)
                       .normalize();
        vec3 right = fwd.cross(up).normalize();
        vec3 camUp = right.cross(fwd);
        float dist = std::sqrt((cp.x - look.x) * (cp.x - look.x)
                             + (cp.y - look.y) * (cp.y - look.y)
                             + (cp.z - look.z) * (cp.z - look.z));
        if (dist < 1e-4f) dist = 1e-4f;
        // Drag direction follows the cursor: moving right pushes the
        // scene right (camera/pivot move left), hence -ndx on right.
        float sx = -ndx * dist;
        float sy =  ndy * dist;
        look.x += right.x * sx + camUp.x * sy;
        look.y += right.y * sx + camUp.y * sy;
        look.z += right.z * sx + camUp.z * sy;
        rotatePosition();
    }
    
    mat4 lookAt() {
        return ::lookAt(curPosition, look, up);
    }
    
    mat4 perspective(float aspect, float zNear, float zFar) {
        aspect = aspect;
        zNear = zNear;
        zFar = zFar;
        return ::perspective(toRadian(fovy), aspect, zNear, zFar);
    }
    mat4 perspective(float aspect) { return perspective(aspect, zNear, zFar); }
    
    void glfwSetCallbacks(GLFWwindow* window) {
        glfwSetCursorPosCallback(window, cursorPosCallback);
        glfwSetScrollCallback(window, scrollCallback);
    }
    
    vec3 getCurPosition() { return curPosition; }


    /// unproject the screen space (x, y) into world space point.
    /// nz: normalized z [-1.0, 1.0] in the perspective frustrum.
    vec3 unProjectPerspective(GLFWwindow* window, double x, double y, double nz) {
        int w, h;
        glfwGetWindowSize(window, &w, &h); // size of the screen coordinate
        double nx =  x/w*2-1;
        double ny = -y/h*2+1;
        
        // z normalized point
        vec4 np(nx, ny, nz, 1);
        mat4 invVP = (::perspective(toRadian(fovy), w/(float)h, zNear, zFar) * ::lookAt(curPosition, look, up)).inverse();
        vec4 p4 = invVP*np;
        return vec3(p4)/p4.w;
    }
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

namespace YGL {

void cursorPosCallback(GLFWwindow *window, double xpos, double ypos)
{
    static double lastX = 0;
    static double lastY = 0;

    bool left  = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
    bool right = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT);
    bool mid   = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE);
    bool shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS
              || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;

    if (left || right || mid)
    {
        int w, h;
        glfwGetWindowSize(window, &w, &h);
        double dx = (xpos - lastX) / w;
        double dy = (ypos - lastY) / h;

        // Pan with right / middle drag, or Shift+left (trackpad-
        // friendly). Plain left drag keeps orbiting as before.
        if (right || mid || (left && shift)) {
            camera.pan((float)dx, (float)dy);
        } else {
            camera.theta -= dx * PI; // related with y-axis rotation
            camera.phi -= dy * PI;   // related with x-axis rotation
            camera.phi = comp::clamp(camera.phi, -PI / 2 + 0.01f, PI / 2 - 0.01f);

            camera.rotatePosition();
        }
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

};

#endif
