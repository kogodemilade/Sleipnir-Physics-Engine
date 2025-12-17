// #include "Sleipnir/world.hpp"
// #include "Sleipnir/body.hpp"
// #include "Sleipnir/links.hpp"
// #include "Sleipnir/collide_fine.hpp"
// #include "Sleipnir/collide_coarse.hpp"
// #include "Sleipnir/core.hpp"
// #include "Sleipnir/precision.hpp"
// #include <iostream>
// #include <thread>
// #include <chrono>
// #include <iostream>
// #include <thread>
// #include <chrono>
// #include <raylib.h>

// static inline ::Vector3 ToRayLib(const cyclone::Vector3& v) {
// return { v.x, v.y, v.z };
// }

// static inline ::Quaternion ToRayLib(const cyclone::Quaternion& q) {
//     return { q.x, q.y, q.z, q.w };
// }

// void DrawRigidBox(
//     const cyclone::RigidBody& body,
//     const cyclone::Vector3& halfSize,
//     Color color
// ) {
//     ::Vector3 pos = ToRayLib(body.getPosition());

//     DrawCubeV(
//         pos,
//         { halfSize.x * 2, halfSize.y * 2, halfSize.z * 2 },
//         color
//     );

//     DrawCubeWiresV(
//         pos,
//         { halfSize.x * 2, halfSize.y * 2, halfSize.z * 2 },
//         BLACK
//     );
// }

// // Draw a line between two rigidbody points
// void DrawRigidLine(
//     const cyclone::Vector3& start,
//     const cyclone::Vector3& end,
//     Color color
// ) {
//     ::Vector3 s = ToRayLib(start);
//     ::Vector3 e = ToRayLib(end);

//     DrawLine3D(s, e, color);
// }

// void DrawRigidSphere(
//     const cyclone::RigidBody& body,
//     float radius,
//     Color color
// ) {
//     ::Vector3 pos = ToRayLib(body.getPosition());

//     // Draw solid sphere
//     DrawSphere(pos, radius, color);

//     // Draw wireframe
//     DrawSphereWires(pos, radius, 16, 16, BLACK); // 16 slices/stacks for decent smoothness
// }

// int main() {
//     /*Start some rendering stuff*/

//     InitWindow(1280, 800, "Cyclone Physics Debug (raylib)");
//     SetTargetFPS(60);

//     Camera3D camera{};
//     camera.position = { 0.0f, 6.0f, 8.0f };
//     camera.target   = { 0.0f, 2.0f, 0.0f };
//     camera.up       = { 0.0f, 1.0f, 0.0f };
//     camera.fovy     = 60.0f;
//     camera.projection = CAMERA_PERSPECTIVE;
//     /*end rendering stuff*/

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
//     b.setAwake(1);
//     b.name = "mass";
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
//     anchor.name = "Anchor";
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

//     /*End rendering code*/
       
// int i = 0;
// const float physics_dt = 0.01f;            
// float accumulator = 0.0f;
// auto previous = std::chrono::high_resolution_clock::now();

// int iter = 0;
// while (!WindowShouldClose()) {
//     auto now = std::chrono::high_resolution_clock::now();
//     float frameTime = std::chrono::duration<float>(now - previous).count();
//     previous = now;

//     // clamp to avoid spiral of death if frameTime huge
//     if (frameTime > 0.25f) frameTime = 0.25f;

//     accumulator += frameTime;

//     // Step physics with fixed substeps,
//     while (accumulator >= physics_dt) {
//         world.startFrame();

//             if( IsKeyDown(KEY_K)){
//         world.runPhysics(physics_dt);}

//         accumulator -= physics_dt;
//         iter++;
//     }

//     if ((iter % 10) == 0) {
//     for (auto b: world.rigidBodies){
//     std::cout << "iteration: " << iter*0.01 << "secs\n";
//     std::cout<<"Body: "<<b->name<<std::endl;
//     std::cout << "Position: x=" << b->getPosition().x
//                 << " y=" << b->getPosition().y << " z="<<b->getPosition().z << "\n";
//     std::cout << "Velocity: x=" << b->getVelocity().x
//                 << " y=" << b->getVelocity().y << " z="<<b->getVelocity().z<<"\n";
//     std::cout << "Orientation: x=" << b->getOrientation().toEulerAngles().x << " y="<<b->getOrientation().toEulerAngles().y << " z="<<b->getOrientation().toEulerAngles().z << "\n";
//     std::cout << "Acceleration: x=" << b->getAcceleration().x
//                 << " y=" << b->getAcceleration().y << "\n\n";
    
//     }}

//     // Rendering section unchanged (use bodies' current transforms)

//     UpdateCamera(&camera, CAMERA_FREE);

//         BeginDrawing();
//         ClearBackground({ 38, 38, 46, 255 });

//         BeginMode3D(camera);

//         DrawPlane({0,0,0}, {40,40}, WHITE);

//         DrawRigidSphere(
//             b,
//             sphere.radius,
//             RED
//         );

//         DrawRigidSphere(
//             anchor,
//             sphereAnchor.radius,
//             GREEN
//         );

//         DrawRigidLine(
//             b.getPosition(),
//             anchor.getPosition(),
//             BLUE
//         );

       
//         EndMode3D();

//         DrawFPS(20, 20);
//         EndDrawing();
// }
//         // cleanup

//     return 0;
// }




// // //////////////////////////////////////////////////////////////
// // #include "Sleipnir/world.hpp"
// // #include "Sleipnir/body.hpp"
// // #include "Sleipnir/links.hpp"
// // #include "Sleipnir/collide_fine.hpp"
// // #include "Sleipnir/collide_coarse.hpp"
// // #include "Sleipnir/core.hpp"
// // #include "Sleipnir/precision.hpp"
// // #include "Yggdrasil/ygg/engine.hpp"
// // #include "Sleipnir/ragdoll.hpp"
// // #include <iostream>
// // #include <thread>
// // #include <chrono>
// // #include <iostream>
// // #include <thread>
// // #include <chrono>

// // using namespace cyclone;
// // int main(){
// // cyclone::World world(1000, 1000);

// // Vector3 ragRoot(0.0f, 6.0f, 0.0f);
// // auto rag = cyclone::RagdollBuilder::createSimpleRagdoll(world, ragRoot);
// // int i = 0;


// // cyclone::RigidBody ground;
// // ground.setMass(0);
// // ground.setPosition(cyclone::Vector3(0,0,0));
// // ground.setSize(20.0f);
// // cyclone::Vector3 groundInertiaX(0.0, 0.0, 0.0);
// // cyclone::Vector3 groundInertiaY(0.0, 0.0, 0.0);
// // cyclone::Vector3 groundInertiaZ(0.0, 0.0, 0.0);
// // cyclone::Matrix3 groundTensor;
// // groundTensor.setComponents(groundInertiaX, groundInertiaY, groundInertiaZ);
// // ground.setInvInertiaTensor(groundTensor);
// // ground.calculateDerivedData();

// // cyclone::Plane groundPlane;
// // groundPlane.body = &ground;
// // groundPlane.normal = cyclone::Vector3(0, 1, 0);
// // groundPlane.offset = (cyclone::real)0;
// // groundPlane.bindPrimitive();
// // groundPlane.calculateInternals();


// // world.addBodies(&ground, 1);
// // cyclone::Gravity gravity(cyclone::Vector3(0, -9.81f, 0));

// // for (auto b: rag.bodies){
// //     world.registry.add(b, &gravity);
// // }

// // while(1){
// //     world.startFrame();
// //     world.runPhysics(0.01);
// //     for (auto b: rag.bodies){
// //             std::cout << "iteration: " << i*0.01 << "secs\n";
// //             std::cout << "Position: x=" << b->getPosition().x
// //                       << " y=" << b->getPosition().y << "\n";
// //             std::cout << "Velocity: x=" << b->getVelocity().x
// //                       << " y=" << b->getVelocity().y << "\n";
// //             std::cout << "Acceleration: x=" << b->getAcceleration().x
// //                       << " y=" << b->getAcceleration().y << "\n\n";
            
// //             // for (auto data: anchor.getInvInertiaTensorWorld().data){
// //             // std::cout << "Inverse Inertia tensor world "<<data<<"\n";}
// //             // std::cout << "\n\n\n\n";
// //         }
// //         i++;
// //     }
// // return 0;
// // }