// #include "Sleipnir/world.hpp"
// #include "Sleipnir/body.hpp"
// #include "Sleipnir/links.hpp"
// #include "Sleipnir/collide_fine.hpp"
// #include "Sleipnir/collide_coarse.hpp"
// #include "Sleipnir/core.hpp"
// #include "Sleipnir/precision.hpp"
// #include "Yggdrasil/ygg/engine.hpp"
// #include <iostream>
// #include <thread>
// #include <chrono>
// #include <iostream>
// #include <thread>
// #include <chrono>

// Ygg::RenderEngine engine;
// glm::mat4 projection;
// Ygg::Camera cam = engine.createCamera({0.0f, 5.0f, 5.0f});

// void framebuffer_size_callback(GLFWwindow *window, int width, int height);
// void mouse_callback(GLFWwindow *window, double xpos, double ypos);
// void scroll_callback(GLFWwindow *window, double xOffset, double yOffset);

// int main() {
//     /*Start some rendering stuff*/
//     if (engine.initGL("../shaders/vShader.glsl", "../shaders/fShader2.glsl") != 0) {
//         return -1;
//     }

//     GLFWwindow *window = engine.getWindow();

//     // simple GL state
//     glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    
//     //register scroll and mouse callback
//     glfwSetCursorPosCallback(window, mouse_callback);
//     glfwSetScrollCallback(window, scroll_callback);

//     auto lastTime = std::chrono::high_resolution_clock::now();

//     /*End some rendering stuff*/
//     cyclone::World world(1000, 1000);

//     // Create pendulum bob
//     cyclone::RigidBody b;
//     b.setMass(50.0f);
//     b.setVelocity(cyclone::Vector3(0, 0, 0));
//     b.setPosition(cyclone::Vector3(10, 5, 0));  // displaced to side
//     b.setLinearDamping(0.99);
//     b.setSize(1);
//     cyclone::real inertia = 0.4 * b.getMass() * b.getSize() * b.getSize();
//     cyclone::Vector3 inertiaX(1.0, 0.0, 0.0);
//     cyclone::Vector3 inertiaY(0.0, 1.0, 0.0);
//     cyclone::Vector3 inertiaZ(0.0, 0.0, 1.0);

//     cyclone::Matrix3 IdentityMatrix;
//     IdentityMatrix.setComponents(inertiaX, inertiaY, inertiaZ);
//     cyclone::Matrix3 inertiaMat = IdentityMatrix * inertia;

//     b.setInertiaTensor(inertiaMat);
//     b.calculateDerivedData();

//     // Create fixed point (anchor)
//     cyclone::RigidBody anchor;
//     anchor.setInverseMass(0.0f);  // immovable
//     anchor.setPosition(cyclone::Vector3(0, 5, 0));
//     anchor.setSize(0.1);
//     cyclone::Vector3 inertiaX_(0, 0, 0);
//     cyclone::Vector3 inertiaY_(0, 0, 0);
//     cyclone::Vector3 inertiaZ_(0, 0, 0);
//     cyclone::Matrix3 maxMatrix;
//     maxMatrix.setComponents(inertiaX_, inertiaY_, inertiaZ_);
//     anchor.setInvInertiaTensor(maxMatrix);
//     anchor.calculateDerivedData();

//     // Connect with a rod (fixed length)
//     cyclone::Cable rod;
//     rod.restitution = 0.0;
//     rod.body[1] = &anchor;
//     rod.body[0] = &b;
//     // rod.rodLength = 10.0f * std::sqrt(2.0f);  // same as current distance
//     rod.maxLength = (b.getPosition() - anchor.getPosition()).magnitude();
//     world.addContactGenerator(&rod);

//     cyclone::Sphere sphere;
//     sphere.body = &b;    
//     sphere.radius = (cyclone::real)1.0;
//     cyclone::Matrix4 offset;
//     offset.setOrientAndPos(cyclone::Quaternion(1, 0, 0, 0), cyclone::Vector3(0, 0, 0));
//     sphere.offset = offset;
//     sphere.bindPrimitive();
//     sphere.calculateInternals();

//     cyclone::Sphere sphereAnchor;
//     sphereAnchor.body = &anchor;
//     sphereAnchor.radius = 0.1;
//     sphereAnchor.offset = offset;
//     sphereAnchor.bindPrimitive();
//     sphereAnchor.calculateInternals();

//     // Create ground
//     cyclone::RigidBody ground;
//     ground.setMass(0);
//     ground.setPosition(cyclone::Vector3(0,0,0));
//     ground.setSize(20.0f);
//     cyclone::Vector3 groundInertiaX(0.0, 0.0, 0.0);
//     cyclone::Vector3 groundInertiaY(0.0, 0.0, 0.0);
//     cyclone::Vector3 groundInertiaZ(0.0, 0.0, 0.0);
//     cyclone::Matrix3 groundTensor;
//     groundTensor.setComponents(groundInertiaX, groundInertiaY, groundInertiaZ);
//     ground.setInvInertiaTensor(groundTensor);
//     ground.calculateDerivedData();

//     cyclone::Plane groundPlane;
//     groundPlane.body = &ground;
//     groundPlane.normal = cyclone::Vector3(0, 1, 0);
//     groundPlane.offset = (cyclone::real)0;
//     groundPlane.bindPrimitive();
//     groundPlane.calculateInternals();



//         // Add gravity
//     cyclone::Gravity gravity(cyclone::Vector3(0, -9.81f, 0));

//     // Add to world
//     world.addBodies(&b);
//     world.addBodies(&anchor);
//     world.addBodies(&ground, 1);
//     world.registry.add(&b, &gravity);

//     /*Start rendering code*/
//     // // create a few demo meshes
//     Ygg::Mesh floor = engine.createBox(ground.getPosition().toGlm(), ground.getOrientation().toGlm(), ground.getSize(), 0.0001f, ground.getSize(), {0.7f, 1.7f, 0.0f}); //Instantiate the rest with correct arguments
//     Ygg::Mesh mass = engine.createSphere(b.getPosition().toGlm(), b.getOrientation().toGlm(), b.getSize(), {0.9f, 0.8f, 0.7f}, 32, 32);
//     Ygg::Mesh anchorMesh = engine.createSphere(anchor.getPosition().toGlm(), anchor.getOrientation().toGlm(), anchor.getSize(), {0.9f, 0.8f, 0.7f}, 32, 32);

//     static Ygg::Line thread = engine.createLine();

//     /*End rendering code*/

//     float duration = 0.01;
//     int i = 0;
//     while (!glfwWindowShouldClose(window)) {
//         auto now = std::chrono::high_resolution_clock::now();
//         float dt = std::chrono::duration<float>(now - lastTime).count();
//         lastTime = now;



//         world.startFrame();
//         world.runPhysics(duration);

//         /*Start rendering code*/
//         cam.processInput(window, dt);

//         // // render
//         glClearColor(0.15f, 0.15f, 0.18f, 1.0f);
//         glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//         // // draw meshes (these meshes were baked with model transforms in createBox/createSphere)
//         glm::mat4 view = cam.getViewMatrix();
//         engine.drawMesh(floor, view, projection, cam.getCameraPos(), floor.model);
//         engine.drawMesh(mass, view, projection, cam.getCameraPos(), b.getPosandOrient().toGlm());
//         engine.drawMesh(anchorMesh, view, projection, cam.getCameraPos(), anchor.getPosandOrient().toGlm());



//         glm::vec3 p1 = anchor.getPosition().toGlm();
//         glm::vec3 p2 = b.getPosition().toGlm();
//         glm::vec3 ropecolor = {0,0,0};
//         engine.updateLine(thread, p1, p2, ropecolor);
//         engine.drawLine(thread, view, projection, cam.getCameraPos());

//         glfwSwapBuffers(window);
//         glfwPollEvents();
//         /*Stop rendering code*/

//         if (i % 1 == 0) {
//             std::cout << "iteration: " << i*duration << "secs\n";
//             std::cout << "Position: x=" << b.getPosition().x
//                       << " y=" << b.getPosition().y << "\n";
//             std::cout << "Velocity: x=" << b.getVelocity().x
//                       << " y=" << b.getVelocity().y << "\n";
//             std::cout << "Acceleration: x=" << b.getAcceleration().x
//                       << " y=" << b.getAcceleration().y << "\n\n";
            
//             // for (auto data: anchor.getInvInertiaTensorWorld().data){
//             // std::cout << "Inverse Inertia tensor world "<<data<<"\n";}
//             // std::cout << "\n\n\n\n";
//         }

//         i++;
//         // std::this_thread::sleep_for(std::chrono::milliseconds(30));
//     }
//         // cleanup
//     engine.cleanupMesh(floor);
//     engine.cleanupMesh(mass);
//     engine.cleanupMesh(anchorMesh);

//     engine.terminate();
//     return 0;
// }


// /*Some other rendering stuff*/
// void framebuffer_size_callback(GLFWwindow *window, int width, int height){
//     glViewport(0, 0, width, height);
//     projection = glm::perspective(glm::radians(cam.getFov()), (float)width/height, 0.1f, 100.0f);

// }

// void mouse_callback(GLFWwindow *window, double xpos, double ypos){
//     cam.processMouseMovement(window, xpos, ypos);
// }

// void scroll_callback(GLFWwindow *window, double xOffset, double yOffset){
//     cam.processMouseScroll(static_cast<float>(yOffset), projection);
// }





// //////////////////////////////////////////////////////////////
// #include "Sleipnir/world.hpp"
// #include "Sleipnir/body.hpp"
// #include "Sleipnir/links.hpp"
// #include "Sleipnir/collide_fine.hpp"
// #include "Sleipnir/collide_coarse.hpp"
// #include "Sleipnir/core.hpp"
// #include "Sleipnir/precision.hpp"
// #include "Yggdrasil/ygg/engine.hpp"
// #include "Sleipnir/ragdoll.hpp"
// #include <iostream>
// #include <thread>
// #include <chrono>
// #include <iostream>
// #include <thread>
// #include <chrono>

// using namespace cyclone;
// int main(){
// cyclone::World world(1000, 1000);

// Vector3 ragRoot(0.0f, 6.0f, 0.0f);
// auto rag = cyclone::RagdollBuilder::createSimpleRagdoll(world, ragRoot);
// int i = 0;


// cyclone::RigidBody ground;
// ground.setMass(0);
// ground.setPosition(cyclone::Vector3(0,0,0));
// ground.setSize(20.0f);
// cyclone::Vector3 groundInertiaX(0.0, 0.0, 0.0);
// cyclone::Vector3 groundInertiaY(0.0, 0.0, 0.0);
// cyclone::Vector3 groundInertiaZ(0.0, 0.0, 0.0);
// cyclone::Matrix3 groundTensor;
// groundTensor.setComponents(groundInertiaX, groundInertiaY, groundInertiaZ);
// ground.setInvInertiaTensor(groundTensor);
// ground.calculateDerivedData();

// cyclone::Plane groundPlane;
// groundPlane.body = &ground;
// groundPlane.normal = cyclone::Vector3(0, 1, 0);
// groundPlane.offset = (cyclone::real)0;
// groundPlane.bindPrimitive();
// groundPlane.calculateInternals();


// world.addBodies(&ground, 1);
// cyclone::Gravity gravity(cyclone::Vector3(0, -9.81f, 0));

// for (auto b: rag.bodies){
//     world.registry.add(b, &gravity);
// }

// while(1){
//     world.startFrame();
//     world.runPhysics(0.01);
//     for (auto b: rag.bodies){
//             std::cout << "iteration: " << i*0.01 << "secs\n";
//             std::cout << "Position: x=" << b->getPosition().x
//                       << " y=" << b->getPosition().y << "\n";
//             std::cout << "Velocity: x=" << b->getVelocity().x
//                       << " y=" << b->getVelocity().y << "\n";
//             std::cout << "Acceleration: x=" << b->getAcceleration().x
//                       << " y=" << b->getAcceleration().y << "\n\n";
            
//             // for (auto data: anchor.getInvInertiaTensorWorld().data){
//             // std::cout << "Inverse Inertia tensor world "<<data<<"\n";}
//             // std::cout << "\n\n\n\n";
//         }
//         i++;
//     }
// return 0;
// }