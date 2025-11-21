#pragma once
#ifndef CAMERA_H
#define CAMERA_H
#include "../glad/glad.h" /*Include glad to get all required opengl headers*/
#include <GLFW/glfw3.h>
#include "../glm/glm.hpp"
#include "../glm/ext.hpp"

namespace Ygg{
enum Camera_movement{
    forward, backward, left, right, up, down
};

//Define a few constants and global vars
inline float FOV = 45.0f;
inline float YAW = -90.0f;
inline float PITCH = 0.0f;
inline float SPEED = 2.5f;
inline float MOUSESENSITIVITY = 0.1;

//Camera class to simulate the effect of a camera 
class Camera{
    private:
        glm::vec3 cameraPos;
        glm::vec3 cameraUp;
        glm::vec3 cameraFront;
        glm::vec3 cameraRight;
        glm::vec3 worldUp;

        //Euler angles
        float yaw;
        float pitch;
        float movementSpeed;
        float mouseSensitivity;
        float fov;
        float maxZoom = 60.0f;
        float minZoom = 1.0f;
        float maxPitch = 89.0f;
        float minPitch = -89.0f;
        bool firstMouse = true;
        const unsigned scr_width = 800;
        const unsigned scr_height = 600;
        float lastMouseX = scr_width/2.0f;
        float lastMouseY = scr_height/2.0f;

    public:
        //Initialize with vectors
        Camera(glm::vec3 pos = glm::vec3(0.0f,0.0f,0.0f), glm::vec3 
        up= glm::vec3(0.0f, 1.0f, 0.0f), float yaw = YAW, float pitch = PITCH) : 
        cameraFront(glm::vec3(0.0f, 0.0f, -1.0f)), 
        movementSpeed(SPEED),
        mouseSensitivity(MOUSESENSITIVITY),
        fov(FOV),
        cameraPos(pos),
        worldUp(up),
        yaw(yaw)
        {
            updateCameraVectors();
        }

        //Initialize position only
        // Camera(glm::vec3 pos):         
        // cameraFront(glm::vec3(0.0f, 0.0f, -1.0f)), 
        // movementSpeed(SPEED),
        // mouseSensitivity(MOUSESENSITIVITY),
        // fov(FOV),
        // cameraPos(pos),
        // worldUp(glm::vec3(0.0f, 1.0f, 0.0f)),
        // yaw(yaw)
        // {
        //     updateCameraVectors();
        // }

        //Initialize with defaults
        Camera() :
        cameraFront(glm::vec3(0.0f, 0.0f, -1.0f)), 
        movementSpeed(SPEED),
        mouseSensitivity(MOUSESENSITIVITY),
        fov(FOV),
        cameraPos(glm::vec3(0.0f, 0.0f, 0.0f)),
        worldUp(glm::vec3(0.0f, 1.0f, 0.0f)),
        yaw(yaw)
        {
            updateCameraVectors();
        }

        //Getters and setters

        //Returns Camera Position
        glm::vec3 getCameraPos() const {
            return cameraPos;
        }

        //Returns cameraFfront 
        glm::vec3 getCameraFront() const {
            return cameraFront;
        }

        glm::vec3 getCameraUp() const {
            return cameraUp;
        }
    
        //Returns the axis in the positive X axis
        glm::vec3 getCameraRight() const {
            return cameraRight;
        }

        //Returns the world's up axis
        glm::vec3 getWorldUp() const {
            return worldUp;
        }

        float getYaw() const {
            return yaw;
        }

        float getPitch() const {
            return pitch;
        }

        float getMoveSpeed() const {
            return movementSpeed;
        }

        float getMouseSensitivity() const {
            return mouseSensitivity;
        }

        float getFov() const {
            return fov;
        }

        float getMaxZoom() const {
            return maxZoom;
        }

        float getMinZoom() const {
            return minZoom;
        }

        float getMaxPitch() const {
            return maxZoom;
        }

        float getMinPitch() const{
            return minZoom;
        }


        void setCameraPos(const glm::vec3 &position) {
            cameraPos = position;
        }

        //Returns cameraFfront 
        void setCameraFront(const glm::vec3 &front) {
            cameraFront = front;
        }

        void setCameraUp(const glm::vec3 &up) {
            cameraUp = up;
        }
    
        //Returns the axis in the positive X axis
        void setRight(const glm::vec3 &right_) {
            cameraRight = right_;
        }

        //Returns the world's up axis
        void setWorldUp(const glm::vec3 &worldUp_) {
            worldUp  = worldUp_;
        }

        void setYaw(float yaw_) {
            yaw  = yaw_;
        }

        void setPitch(float pitch_) {
            pitch  = pitch_;
        }

        void setMoveSpeed(float move_speed) {
            movementSpeed  = move_speed;
        }

        void setMouseSensitivity(float sensitivity) {
            mouseSensitivity  = sensitivity;
        }

        void setFov(float fov_) {
            fov= fov_;
        }

        //Max zoom (fov) defaults to 60.0f
        void setMaxZoom(float maxZoom_){
            maxZoom = maxZoom_;
        }

        //Min zoom (fov) defaults to 1.0f
        void setMinZoom(float minZoom_){
            minZoom = minZoom_;
        }

        //Max pitch defaults to 89.0
        void setMaxPitch(float maxPitch_){
            maxPitch = maxPitch_;
        }

        //Min pitch defaults to -89.0
        void setMinPitch(float minPitch_){
            minPitch = minPitch_;
        }

        /*Returns the view matric calculated using euler angles*/
        glm::mat4 getViewMatrix() const{
            return glm::lookAt(cameraPos, cameraPos+cameraFront, cameraUp);
        }

        void processInput(GLFWwindow* window, float deltaTime){
            if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
                glfwSetWindowShouldClose(window, true);

            if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
                processKeyboard(forward, deltaTime);

            if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
                processKeyboard(backward, deltaTime);

            if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
                processKeyboard(left, deltaTime);

            if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
                processKeyboard(right, deltaTime);

            if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
                processKeyboard(up, deltaTime);

            if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
                processKeyboard(down, deltaTime);
        }

        void processMouseMovement(GLFWwindow *window, double xpos, double ypos, GLboolean constrainPitch = true) {
            if (firstMouse) {
                lastMouseX = xpos;
                lastMouseY = ypos;
                firstMouse = false;
            }

            float xOffset = xpos - lastMouseX;
            float yOffset = lastMouseY - ypos; //reversed cuz y coor range from bottom to top
            lastMouseX = xpos;
            lastMouseY = ypos;

            xOffset *= mouseSensitivity;
            yOffset *= mouseSensitivity;
            yaw += xOffset;
            pitch += yOffset;

            //Make sure that when pitch is out of bounds screen doesn't get flipped
            if (constrainPitch){
                if (pitch > maxPitch){
                    pitch = maxPitch;
                } 
                if (pitch < minPitch) {
                    pitch = minPitch;
                }
                //update front, right and up vectors
                updateCameraVectors();
            }
        }

        void processMouseScroll(float yOffset, glm::mat4& projection){
            fov -= (float)yOffset;
            if (fov > maxZoom) {
                fov = maxZoom;
            }
            if (fov < minZoom) {
                fov = minZoom;
            }
            //recalculate projection matrix
            projection = glm::perspective(glm::radians(fov), (float)scr_width/scr_height, 1.0f, 100.0f);

        }


    private:
    //Calculates the front vector from the camera's updates euler angles
        void updateCameraVectors(){
            //Calculate the new front vector
            cameraFront = glm::normalize(glm::vec3(cos(glm::radians(yaw)) * cos(glm::radians(pitch)), 
                                sin(glm::radians(pitch)),
                                sin(glm::radians(yaw)*cos(glm::radians(pitch)))));

            /*regenerate other axes*/
            cameraRight = glm::normalize(glm::cross(cameraFront, worldUp));
            cameraUp = glm::normalize(glm::cross(cameraRight, cameraFront));
        }


        void processKeyboard(Camera_movement direction, float deltaTime) {
            float velocity = movementSpeed * deltaTime;
            if (direction == forward) cameraPos += cameraFront* velocity;
            if (direction == backward) cameraPos -= cameraFront* velocity;
            if (direction == left) cameraPos -= cameraRight* velocity;
            if (direction == right) cameraPos += cameraRight* velocity;
            if (direction == up) cameraPos += cameraUp*velocity;
            if (direction == down) cameraPos -= cameraUp*velocity;
        }
};
};
#endif