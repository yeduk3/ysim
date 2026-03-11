#ifndef YGL_ERROR_HPP
#define YGL_ERROR_HPP

#include <GL/glew.h>

#include <iostream>

/** Get error log
 
 GL_INVALID_ENUM 0x0500
 GL_INVALID_VALUE 0x0501
 GL_INVALID_OPERATION 0x0502
 GL_STACK_OVERFLOW 0x0503
 GL_STACK_UNDERFLOW 0x0504
 GL_OUT_OF_MEMORY 0x0505
 */
bool glErr(const std::string& message) {
    GLint err = glGetError();
    if (err != GL_NO_ERROR) {
        std::string errType;
        switch (err) {
            case GL_INVALID_ENUM:
                errType = "GL_INVALID_ENUM";
                break;
            case GL_INVALID_VALUE:
                errType = "GL_INVALID_VALUE";
                break;
            case GL_INVALID_OPERATION:
                errType = "GL_INVALID_OPERATION";
                break;
            case GL_STACK_OVERFLOW:
                errType = "GL_STACK_OVERFLOW";
                break;
            case GL_STACK_UNDERFLOW:
                errType = "GL_STACK_UNDERFLOW";
                break;
            case GL_OUT_OF_MEMORY:
                errType = "GL_OUT_OF_MEMORY";
                break;
            default:
                break;
        }
        printf("%08X(%s) ", err, errType.c_str());
        std::cerr << "GL Error: " << message << std::endl;
        return true;
    }
    return false;
}

#endif
