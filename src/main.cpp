#include "Sleipnir/world.hpp"
#include "Sleipnir/body.hpp"
#include "Sleipnir/links.hpp"
#include "Sleipnir/collide_fine.hpp"
#include "Sleipnir/collide_coarse.hpp"
#include "Sleipnir/core.hpp"
#include "Sleipnir/precision.hpp"
#include "Yggdrasil/ygg/engine.hpp"
#include <iostream>
#include <thread>
#include <chrono>


Ygg::RenderEngine engine;
glm::mat4 projection;
Ygg::Camera cam = engine.createCamera({00.0f, 3.0f, 0.0f});

void framebuffer_size_callback(GLFWwindow *window, int width, int height);
void mouse_callback(GLFWwindow *window, double xpos, double ypos);
void scroll_callback(GLFWwindow *window, double xOffset, double yOffset);

int main() {
    /*Start some rendering stuff*/
    if (engine.initGL("../shaders/vShader.glsl", "../shaders/fShader2.glsl") != 0) {
        return -1;
    }

    GLFWwindow *window = engine.getWindow();

    // simple GL state
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    
    //register scroll and mouse callback
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);

    auto lastTime = std::chrono::high_resolution_clock::now();

    /*End some rendering stuff*/




    cyclone::World world(1000, 1000);

    // Create mass
    cyclone::RigidBody b;
    b.setMass(5.0f);
    b.setVelocity(cyclone::Vector3(0, 0, 0));
    b.setPosition(cyclone::Vector3(0, 8, -5)); 
    b.setOrientation(cyclone::Quaternion(1,1,2,1));
    b.setLinearDamping(0.99);
    b.setSize(1);
    cyclone::real inertia = 0.4 * b.getMass() * b.getSize() * b.getSize();
    cyclone::Vector3 inertiaX(1.0, 0.0, 0.0);
    cyclone::Vector3 inertiaY(0.0, 1.0, 0.0);
    cyclone::Vector3 inertiaZ(0.0, 0.0, 1.0);

    cyclone::Matrix3 IdentityMatrix;
    IdentityMatrix.setComponents(inertiaX, inertiaY, inertiaZ);
    cyclone::Matrix3 inertiaMat = IdentityMatrix * inertia;

    b.setInertiaTensor(inertiaMat);
    b.calculateDerivedData();

    cyclone::Sphere sphere;
    sphere.body = &b;    
    sphere.radius = (cyclone::real)1.0;
    cyclone::Matrix4 offset;
    offset.setOrientAndPos(cyclone::Quaternion(1, 0, 0, 0), cyclone::Vector3(0, 0, 0));
    sphere.offset = offset;
    sphere.bindPrimitive();
    sphere.calculateInternals();

    //Create ground
    cyclone::RigidBody ground;
    ground.setMass(0);
    ground.setPosition(cyclone::Vector3(0,2,0));
    ground.setSize(20.0f);
    cyclone::Vector3 groundInertiaX(0.0, 0.0, 0.0);
    cyclone::Vector3 groundInertiaY(0.0, 0.0, 0.0);
    cyclone::Vector3 groundInertiaZ(0.0, 0.0, 0.0);
    cyclone::Matrix3 groundTensor;
    groundTensor.setComponents(groundInertiaX, groundInertiaY, groundInertiaZ);
    ground.setInvInertiaTensor(groundTensor);

    cyclone::Plane groundPlane;
    groundPlane.body = &ground;
    groundPlane.normal = cyclone::Vector3(0, 1, 0);
    groundPlane.offset = (cyclone::real)2;
    groundPlane.bindPrimitive();
    groundPlane.calculateInternals();



     // Add gravity
    cyclone::Gravity gravity(cyclone::Vector3(0, -9.81f, 0));

    // Add to world
    world.addBodies(&b);
    // world.addBodies(&c);
    world.addBodies(&ground, 1);
    world.registry.add(&b, &gravity);
    // world.registry.add(&c, &gravity);


    /*Start rendering code*/
    // create a few demo meshes
    Ygg::Mesh floor = engine.createBox(ground.getPosition().toGlm(), ground.getOrientation().toGlm(), ground.getSize(), 0.0001f, ground.getSize(), {0.7f, 1.7f, 0.0f}); //Instantiate the rest with correct arguments
    Ygg::Mesh head = engine.createSphere(b.getPosition().toGlm(), b.getOrientation().toGlm(), b.getSize(), {0.9f, 0.8f, 0.7f}, 32, 32);
    /*End rendering code*/





    float duration = 0.01;
    int i = 0;
    while (!glfwWindowShouldClose(window)) {
        auto now = std::chrono::high_resolution_clock::now();
        float dt = std::chrono::duration<float>(now - lastTime).count();
        lastTime = now;


        world.startFrame();
        // b.addForceAtBodyPoint(cyclone::Vector3(10, 0, 0), cyclone::Vector3(-2, -2, 0));
        world.runPhysics(dt);


        /*Start rendering code*/
        cam.processInput(window, dt);

        // render
        glClearColor(0.15f, 0.15f, 0.18f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // draw meshes (these meshes were baked with model transforms in createBox/createSphere)
        glm::mat4 view = cam.getViewMatrix();
        engine.drawMesh(floor, view, projection, cam.getCameraPos(), floor.model);
        engine.drawMesh(head, view, projection, cam.getCameraPos(), b.getPosandOrient().toGlm()); //b.getPosandOrient().toGlm()

        glfwSwapBuffers(window);
        glfwPollEvents();
        /*Stop rendering code*/





        if (i % 10 == 0) {
            std::cout << "iteration: " << i*duration << "secs\n";
            std::cout<<"body1\n";
            std::cout << "Position: x=" << b.getPosition().x
                      << " y=" << b.getPosition().y << "\n";
            std::cout << "Velocity: x=" << b.getVelocity().x
                      << " y=" << b.getVelocity().y << "\n";
            std::cout << "Acceleration: x=" << b.getAcceleration().x
                      << " y=" << b.getAcceleration().y << "\n";

            std::cout << "Orientation: x=" << b.getOrientation().toEulerAngles().x
                      << " y=" << b.getOrientation().toEulerAngles().y
                      << " z=" << b.getOrientation().toEulerAngles().z << "\n";
            std::cout << "Rotation: x=" << b.getRotation().x
                      << " y=" << b.getRotation().y << "\n\n";
            
            // for (auto data: anchor.getInvInertiaTensorWorld().data){
            // std::cout << "Inverse Inertia tensor world "<<data<<"\n";}
            // std::cout << "\n\n\n\n";
        }

        i++;
        // std::cout<<dt<<std::endl;
        // std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }
    
        // cleanup
    engine.cleanupMesh(floor);
    engine.cleanupMesh(head);

    engine.terminate();
    return 0;
}


/*Some other rendering stuff*/
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