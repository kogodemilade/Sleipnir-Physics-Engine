
#pragma once
#include "precision.hpp"
#include "../glad/glad.h"
#include "../utils/shader.hpp"
#include "../utils/camera.hpp"
#include <GLFW/glfw3.h>
#include "../glm/glm.hpp"
#include "../glm/ext.hpp"
#include <iostream>
#include <vector>
#include <cmath>

#define Program Shader

namespace Ygg {

struct Mesh {
    unsigned int VAO = 0, VBO = 0, EBO = 0;
    unsigned int indexCount = 0;
    glm::mat4 model;
    std::vector<float> normals;
};

struct Vertex {
    glm::vec3 pos;
    glm::vec3 normal;
    glm::vec3 color;
};

class RenderEngine {
private:
    static GLFWwindow *window;
    Program program;

    const unsigned int SCR_WIDTH = 800;
    const unsigned int SCR_HEIGHT = 600;

    static void framebuffer_size_callback(GLFWwindow *window, int width, int height) {
        glViewport(0, 0, width, height);
    }

    glm::vec3 unit_box[8] = {
        {-0.5f, -0.5f, -0.5f}, {0.5f, -0.5f, -0.5f}, {0.5f, 0.5f, -0.5f}, {-0.5f, 0.5f, -0.5f},
        {-0.5f, -0.5f,  0.5f}, {0.5f, -0.5f,  0.5f}, {0.5f, 0.5f,  0.5f}, {-0.5f, 0.5f,  0.5f}
    };

    glm::vec3 box_face_normals[6] = {
        {1,0,0}, {-1,0,0},
        {0,1,0}, {0,-1,-0},
        {0,0,1}, {0,0,-1}
    };

    unsigned int unit_indices[36] = {
        0,1,2, 2,3,0,  4,5,6, 6,7,4,
        0,4,7, 7,3,0,  1,5,6, 6,2,1,
        0,1,5, 5,4,0,  3,2,6, 6,7,3
    };

public:
    // initGL will create the GLFW window, load GLAD and compile shaders
    int initGL(const char *vShader = "../shaders/vshader.glsl", const char *fShader = "../shaders/fshader.glsl");

    RenderEngine(){}

    GLFWwindow* getWindow();

    Camera createCamera(glm::vec3 pos = {0.0f, 0.0f, 3.0f});

    // createBox and createSphere create Mesh objects whose vertex positions are already transformed (simple)
    Mesh createBox(const glm::vec3 &pos, const glm::quat &orientation,
                   float width, float height, float depth, const glm::vec3 &color);

    Mesh createSphere(const glm::vec3 &pos, const glm::quat &orientation,
                      float radius, const glm::vec3 &color,
                      unsigned int stacks = 12, unsigned int slices = 12);

    // drawing, cleanup, termination utilities
    void drawMesh(const Mesh &mesh,  const glm::mat4& view,  const glm::mat4& projection, const glm::vec3 &cameraPos, const glm::mat4 &rotAndPos);
    void cleanupMesh(Mesh &mesh);
    void terminate();

    // set camera uniforms (view/proj) before drawing your scene
    void setCameraUniforms(const Camera &cam);
};

} // namespace Ygg

