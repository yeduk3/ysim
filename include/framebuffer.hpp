//
//  framebuffer.hpp
//  ObjectiveOpenGL
//
//  Created by 이용규 on 3/17/25.
//

#ifndef framebuffer_hpp
#define framebuffer_hpp

#include "GL/glew.h"
#include "GLFW/glfw3.h"
#include <stdio.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#include <error.hpp>

#include <iostream>
using std::cerr;
using std::endl;

struct TextureFormat {
    /** Specifies the number of color components in the texture.
     */
    GLint internalFormat;
    /** Specifies the format of the pixel data
     
     */
    GLenum format;
    /** Specifies the data type of the pixel data.
     
     */
    GLenum type;
    
    /** Generate texture format data by internalFormat
    
     - Parameters:
        - parameter internalFormat: Split this into format and type value.
     */
    void generate(const GLint &internalFormat) {
        this->internalFormat = internalFormat;
        if (internalFormat == GL_RGBA32F) {
            this->format = GL_RGBA;
            this->type = GL_FLOAT;
        }
    }
};

/** Framebuffer managing object.
 */
struct Framebuffer {
    /** Framebuffer object's ID(handle)*/
    GLuint id = 0;
    int width = 0, height = 0;
    
    GLuint renderbuffer = 0;
    std::vector<GLuint> textureIDs = {};
    std::vector<GLuint> drawBufs = {};
    
    bool depthTest = true;
    
    /** Generate a new framebuffer object.
     
     Generate a new framebuffer object with given width and height.
     
     - Parameters:
         - parameter w: **Width** of the framebuffer
         - parameter h: **Height** of the framebuffer
     */
    void init(GLFWwindow *window, const int w_ = -1, const int h_ = -1) {
        this->cleanup();
        
        if(w_ == -1 || h_ == -1) {
            int w, h;
            glfwGetFramebufferSize(window, &w, &h);
            this->width  = w;
            this->height = h;
        } else {
            this->width  = w_;
            this->height = h_;
        }
        
        glGenFramebuffers(1, &(this->id));
        
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            cerr << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << endl;
    }

    /** Set a default framebuffer object.
     
     Set a default framebuffer object with given width and height.
     
     - Parameters:
         - parameter w: **Width** of the default framebuffer
         - parameter h: **Height** of the default framebuffer
     */
    void initDefault(GLFWwindow *window) {
        this->cleanup();
        
        int w, h;
        glfwGetFramebufferSize(window, &w, &h);
        this->width = w;
        this->height = h;
        
        this->id = 0;
    }
    
    
    /** Generate new 2D textures and attach to this framebuffer.
     
     Generated textures are canvas of this framebuffer's draw call.
     
     Used functions:
     
     - [glGenTextures](https://registry.khronos.org/OpenGL-Refpages/gl4/html/glGenTextures.xhtml)
     
     - [glTexImage2D](https://registry.khronos.org/OpenGL-Refpages/gl4/html/glTexImage2D.xhtml)
     
     - Parameters:
     
        - parameter nTexture: Number of the textures will be attached.
     
        - parameter format: Texture's format used when creating texture.
     */
    void attachTexture2D(const int &nTexture, const TextureFormat &format, int width = -1, int height = -1) {
        if (width  < 0) width  = this->width;
        if (height < 0) height = this->height;

        this->bind();
        
        for (int i = 0; i < nTexture; i++) {
            auto curidx = this->textureIDs.size();
            this->textureIDs.push_back(-1);
            glGenTextures(1, &this->textureIDs[curidx]);
            glBindTexture(GL_TEXTURE_2D, this->textureIDs[curidx]);
            glTexImage2D(GL_TEXTURE_2D,
                         0,
                         format.internalFormat,
                         this->width,
                         this->height,
                         0,
                         format.format,
                         format.type,
                         0);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            auto drawUnit = GL_COLOR_ATTACHMENT0 + int(curidx);
            drawBufs.push_back(drawUnit);
            glFramebufferTexture2D(GL_FRAMEBUFFER,
                                   drawUnit,
                                   GL_TEXTURE_2D,
                                   this->textureIDs[curidx],
                                   0);
            glErr("Error on Attaching Texture2D onto Framebuffer.");
        }
        glDrawBuffers(int(drawBufs.size()), drawBufs.data());
        glErr("Error on attachTexture2D()");
        
        // unbind all
    //    glBindTexture(GL_TEXTURE_2D, 0);
        
        
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;
        
        
        this->unbind();
    }

    
    /** Generate new 2D textures and attach to this framebuffer.
     
     Generated textures are canvas of this framebuffer's draw call.
     
     Used functions:
     
     - [glGenTextures](https://registry.khronos.org/OpenGL-Refpages/gl4/html/glGenTextures.xhtml)
     
     - [glTexImage2D](https://registry.khronos.org/OpenGL-Refpages/gl4/html/glTexImage2D.xhtml)
     
     - Parameters:
     
        - parameter nTexture: Number of the textures will be attached.
     
        - parameter internalFormat: Texture's internalFormat used when create TextureFormat object and call again.
     */
    void attachTexture2D(const int nTexture, const GLint internalFormat, const int width = -1, const int height = -1) {
    //    this->bind();
        
        TextureFormat tf;
        tf.generate(internalFormat);
        
        this->attachTexture2D(nTexture, tf, width, height);
        
        this->unbind();
    }
    
    
    int loadTexture2D(const char *fileName, int &width, int &height) {
        int bytesPerPixel;
        stbi_set_flip_vertically_on_load(true);
        unsigned char *data = stbi_load(fileName,
                                        &width,
                                        &height,
                                        &bytesPerPixel,
                                        4);
        
        textureIDs.push_back(-1);
        int tid = int(textureIDs.size())-1;
        
        if( data != nullptr ) {
            glGenTextures(1, &textureIDs[tid]);
            glBindTexture(GL_TEXTURE_2D, textureIDs[tid]);
    #ifdef __APPLE__
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
    #else
            glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA8, width, height);
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, data);
    #endif
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

            stbi_image_free(data);
        } else {
            printf("Texture <%s> not found.\n", fileName);
            return -1;
        }
        glActiveTexture(GL_TEXTURE0 + tid);
        return tid;
    }

    void attachRenderBuffer(const GLenum internalFormat) {
        this->bind();
        
        glGenRenderbuffers(1, &(this->renderbuffer));
        glBindRenderbuffer(GL_RENDERBUFFER, this->renderbuffer);
        glRenderbufferStorage(GL_RENDERBUFFER,
                              internalFormat,
                              this->width,
                              this->height);
        
        GLenum attachment;
        if(internalFormat == GL_DEPTH24_STENCIL8)
            attachment = GL_DEPTH_STENCIL_ATTACHMENT;
        else if(internalFormat == GL_DEPTH_COMPONENT24)
            attachment = GL_DEPTH_ATTACHMENT;
        else
            return;
        glFramebufferRenderbuffer(GL_FRAMEBUFFER,
                                  attachment,
                                  GL_RENDERBUFFER,
                                  this->renderbuffer);
        

        glBindRenderbuffer(GL_RENDERBUFFER, 0);
        this->unbind();
    }

    void render(GLFWwindow* window, const GLuint vao) {
        this->bind();
    //    std::cout << "render id : " << this->id << std::endl;
        
        glViewport(0, 0, this->width, this->height);
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT);
        
        glBindVertexArray(vao);
        
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 8);
        
        this->unbind();
    }
    
    void render(GLFWwindow* window, const GLuint vao, const GLuint veo, const GLsizei count) {
        this->bind();
    //    std::cout << "render id : " << this->id << std::endl;
        
        glViewport(0, 0, this->width, this->height);
        glClearColor(0, 0, 0, 0);
        if (depthTest) {
            glEnable(GL_DEPTH_TEST);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        } else {
            glClear(GL_COLOR_BUFFER_BIT);
        }
        
        glBindVertexArray(vao);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, veo);
        
        glDrawElements(GL_TRIANGLES, count, GL_UNSIGNED_SHORT, 0);
        
        this->unbind();
    }
    
    void renderFullScreenQuad(const bool finalRender = false) {
        if(finalRender) this->unbind();
        else            this->bind();
        
        if(fsQuadVao == -1) {
            std::vector<float> fsQuad = {
                // positions   // texCoords
                -1.0f, 1.0f, 0.0f, 1.0f,
                -1.0f, -1.0f, 0.0f, 0.0f,
                1.0f, -1.0f, 1.0f, 0.0f,
                
                -1.0f, 1.0f, 0.0f, 1.0f,
                1.0f, -1.0f, 1.0f, 0.0f,
                1.0f, 1.0f, 1.0f, 1.0f
            };
            
            glGenVertexArrays(1, &fsQuadVao);
            glBindVertexArray(fsQuadVao);
            
            glGenBuffers(1, &fsQuadVbo);
            glBindBuffer(GL_ARRAY_BUFFER, fsQuadVbo);
            glBufferData(GL_ARRAY_BUFFER, 24 * sizeof(float), fsQuad.data(), GL_STATIC_DRAW);
            
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), 0);
            glEnableVertexAttribArray(2);
            glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)(2 * sizeof(float)));
        }
        glBindVertexArray(fsQuadVao);
        glDrawArrays(GL_TRIANGLES, 0, 6);
        
        this->unbind();
    }
    
    void setDrawBuffers(const std::initializer_list<GLuint>& indices) {
        this->bind();
        std::vector<GLuint> dBuf(drawBufs.size(), GL_NONE);
        for (auto i : indices) dBuf[i] = drawBufs[i];
        glDrawBuffers(int(dBuf.size()), dBuf.data());
    }

//private:
    /** Bind this framebuffer object.*/
    void bind() {
        glBindFramebuffer(GL_FRAMEBUFFER, this->id);
    //    std::cout << "Framebuffer Bounded: " << this->id << std::endl;
    }
    /** Unbind this framebuffer object.*/
    void unbind() {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    //    std::cout << "Framebuffer Unbounded: " << this->id << std::endl;
    }

    void cleanup() {
        if (this->id != 0) {
            glDeleteFramebuffers(1, &this->id);
        }
        for (auto i = 0; i < textureIDs.size(); i++) {
            glDeleteTextures(1, &textureIDs[i]);
        }
        if (this->renderbuffer != 0) {
            glDeleteRenderbuffers(1, &this->renderbuffer); // 렌더버퍼 삭제
        }
    }
    
private:
    GLuint fsQuadVao = -1, fsQuadVbo;
    

public:
    ~Framebuffer() {
        cleanup();
    }
};

#endif /* framebuffer_hpp */
