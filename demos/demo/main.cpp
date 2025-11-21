// example_main.cpp
#include "Yggdrasil/ygg/engine.hpp"
#include <chrono>

Ygg::RenderEngine engine;
glm::mat4 projection;
Ygg::Camera cam = engine.createCamera({0.0f, 1.5f, 6.0f});

void framebuffer_size_callback(GLFWwindow *window, int width, int height);
void mouse_callback(GLFWwindow *window, double xpos, double ypos);
void scroll_callback(GLFWwindow *window, double xOffset, double yOffset);

int main() {

    if (engine.initGL("shaders/vShader.glsl", "shaders/fShader2.glsl") != 0) {
        return -1;
    }

    GLFWwindow *window = engine.getWindow();

    // create a few demo meshes
    Ygg::Mesh floor = engine.createBox({0.0f, -1.0f, 0.0f}, glm::quat(), 10.0f, 1.0f, 10.0f, {0.7f, 0.7f, 0.7f});
    Ygg::Mesh torso = engine.createBox({0.0f, 0.5f, 0.0f}, glm::quat(), 0.6f, 0.9f, 0.3f, {0.8f, 0.3f, 0.3f});
    Ygg::Mesh head = engine.createSphere({0.0f, 1.3f, 0.0f}, glm::quat(), 0.22f, {0.9f, 0.8f, 0.7f}, 16, 16);
    Ygg::Mesh leftUpperArm = engine.createBox({-0.7f, 0.6f, 0.0f}, glm::quat(), 0.2f, 0.5f, 0.2f, {0.3f, 0.3f, 0.8f});
    Ygg::Mesh rightUpperArm = engine.createBox({0.7f, 0.6f, 0.0f}, glm::quat(), 0.2f, 0.5f, 0.2f, {0.3f, 0.3f, 0.8f});

    // simple GL state
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    
    //register scroll and mouse callback
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);

    auto lastTime = std::chrono::high_resolution_clock::now();

    float i = 1;
    while (!glfwWindowShouldClose(window)) {
        i+=2;
        // delta time
        auto now = std::chrono::high_resolution_clock::now();
        float dt = std::chrono::duration<float>(now - lastTime).count();
        lastTime = now;

        // input
        // mouse_callback();
        cam.processInput(window, dt);
        // update camera (if your Camera has a process input / update API, call it here)

        // render
        glClearColor(0.15f, 0.15f, 0.18f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // engine.setCameraUniforms(cam);
        float s = 1.0f + sin(i) * 0.5f;
        glm::mat4 scaled = torso.model * glm::scale(glm::mat4(1.0f), glm::vec3(s));

        // draw meshes (these meshes were baked with model transforms in createBox/createSphere)
        glm::mat4 view = cam.getViewMatrix();
        engine.drawMesh(floor, view, projection, cam.getCameraPos(), floor.model);
        engine.drawMesh(torso, view, projection, cam.getCameraPos(), torso.model);
        engine.drawMesh(head, view, projection, cam.getCameraPos(), head.model);
        engine.drawMesh(leftUpperArm, view, projection, cam.getCameraPos(), leftUpperArm.model);
        engine.drawMesh(rightUpperArm, view, projection, cam.getCameraPos(), rightUpperArm.model);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // cleanup
    engine.cleanupMesh(floor);
    engine.cleanupMesh(torso);
    // engine.cleanupMesh(head);
    engine.cleanupMesh(leftUpperArm);
    engine.cleanupMesh(rightUpperArm);

    engine.terminate();
    return 0;
}

void framebuffer_size_callback(GLFWwindow *window, int width, int height){
    glViewport(0, 0, width, height);
    projection = glm::perspective(glm::radians(cam.getFov()), (float)width/height, 0.1f, 100.0f);

}

void mouse_callback(GLFWwindow *window, double xpos, double ypos){
    cam.processMouseMovement(window, xpos, ypos);
}

void scroll_callback(GLFWwindow *window, double xOffset, double yOffset){
    cam.processMouseScroll(static_cast<float>(yOffset), projection);
}
