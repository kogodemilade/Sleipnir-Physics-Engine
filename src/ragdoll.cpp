// #include "Sleipnir/world.hpp"
// #include "Sleipnir/body.hpp"
// #include "Sleipnir/links.hpp"
// #include "Sleipnir/collide_fine.hpp"
// #include "Sleipnir/collide_coarse.hpp"
// #include "Sleipnir/core.hpp"
// #include "Sleipnir/precision.hpp"
// // #include "Yggdrasil/ygg/engine.hpp"
// #include "Sleipnir/ragdoll.hpp"
// #include <iostream>
// #include <thread>
// #include <chrono>
// #include <iostream>
// #include <thread>
// #include <chrono>
// #include <raylib.h>
// #include <rlgl.h>

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
//     // ::Vector3 pos = ToRayLib(body.getPosition());
//     // cyclone::Quaternion q = body.getOrientation();
    
//     cyclone::Matrix4 transform = body.getPosandOrient(); 
    
//     // Convert to raylib Matrix (4x4, column-major)
//     // raylib uses column-major, so we need to transpose the rotation part
//     ::Matrix rayMatrix = {
//         transform.data[0], transform.data[4], transform.data[8],  0.0f,  // First COLUMN: r00, r10, r20
//         transform.data[1], transform.data[5], transform.data[9],  0.0f,  // Second COLUMN: r01, r11, r21
//         transform.data[2], transform.data[6], transform.data[10], 0.0f,  // Third COLUMN: r02, r12, r22
//         transform.data[3], transform.data[7], transform.data[11], 1.0f   // Fourth COLUMN: tx, ty, tz
//     };
//     // Begin 3D mode with custom transform
//     rlPushMatrix();
//     rlMultMatrixf((float*)&rayMatrix);
    
//     // Draw the cube centered at origin
//     DrawCube({0, 0, 0}, halfSize.x * 2, halfSize.y * 2, halfSize.z * 2, color);
//     DrawCubeWires({0, 0, 0}, halfSize.x * 2, halfSize.y * 2, halfSize.z * 2, BLACK);
    
//     rlPopMatrix();
// }


// using namespace cyclone;
// int main(){

// InitWindow(1280, 800, "Cyclone Physics Debug (raylib)");
// SetTargetFPS(60);

// Camera3D camera{};
// camera.position = { 0.0f, 6.0f, 8.0f };
// camera.target   = { 0.0f, 2.0f, 0.0f };
// camera.up       = { 0.0f, 1.0f, 0.0f };
// camera.fovy     = 60.0f;
// camera.projection = CAMERA_PERSPECTIVE;

// World world(1000, 1000);
// cyclone::Gravity gravity(cyclone::Vector3(0, -9.81f, 0));

// //Convenience
// Matrix3 IdentityMatrix, ZeroMatrix3;
// IdentityMatrix.identityMatrix();
// ZeroMatrix3.setZero();

// Matrix4 ZeroMatrix4;
// ZeroMatrix4.setZero();


// /*Start some rendering stuff*/

// auto lastTime = std::chrono::high_resolution_clock::now();

// /*End some rendering stuff*/

// // Create ground
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
// ground.name = "ground"; //Change to getter/setter
// ground.setState(0);
// ground.calculateDerivedData();

// cyclone::Plane groundPlane;
// groundPlane.body = &ground;
// groundPlane.normal = cyclone::Vector3(0, 1, 0);
// groundPlane.offset = (cyclone::real)0;
// groundPlane.bindPrimitive();
// groundPlane.calculateInternals();

// world.addBodies(&ground, 1);

// //Create each body. For now, only head, torso, legs and arms.
// //Right Leg
// cyclone::Vector3 rLegPos = cyclone::Vector3(0,0.44,0);
// // cyclone::Vector3 rLegPos = cyclone::Vector3(0,10.5,0);
// cyclone::Quaternion rLegOrient = cyclone::Quaternion(1,0,0,0);
// RigidBody rLeg((real)15.0, rLegPos, rLegOrient, (real)0.9);
// rLeg.setDimension(cyclone::Vector3(0.12f, 0.12f, 0.88f));   //z,x,y ,, d,w,h
// real rLegix = rLeg.getDimension('h')*rLeg.getDimension('h') + rLeg.getDimension('d')*rLeg.getDimension('d');
// real rLegiy = rLeg.getDimension('w')*rLeg.getDimension('w') + rLeg.getDimension('d')*rLeg.getDimension('d');
// real rLegiz = rLeg.getDimension('w')*rLeg.getDimension('w') + rLeg.getDimension('h')*rLeg.getDimension('h');

// Matrix3 rLegInertiaMatrix;
// rLegInertiaMatrix.setDiagonal(rLegix, rLegiy, rLegiz);
// rLegInertiaMatrix = rLegInertiaMatrix * (rLeg.getMass()/12);
// rLeg.setInertiaTensor(rLegInertiaMatrix);
// rLeg.name = "Right Leg";
// rLeg.setSleepEpsilon(0.02);
// rLeg.calculateDerivedData();

// cyclone::Box rLegBox;
// rLegBox.body = &rLeg;    
// rLegBox.halfSize = cyclone::Vector3(0.06, 0.44, 0.06);
// Matrix4 offset;

// offset.setOrientAndPos(ZeroMatrix3, cyclone::Vector3(0,0,0));
// rLegBox.offset = offset;
// rLegBox.calculateInternals();
// rLegBox.bindPrimitive();
// rLeg.setAwake(1);
// world.addBodies(&rLeg);
// world.registry.add(&rLeg, &gravity);


// // Left Leg
// cyclone::Vector3 lLegPos = cyclone::Vector3(0.15,0.44,0);
// cyclone::Quaternion lLegOrient = cyclone::Quaternion(1,0,0,0);
// RigidBody lLeg((real)15, lLegPos, lLegOrient, (real)0.9);
// lLeg.setDimension(cyclone::Vector3(0.12f, 0.12f, 0.88f));
// real lLegix = lLeg.getDimension('h')*lLeg.getDimension('h') + lLeg.getDimension('d')*lLeg.getDimension('d');
// real lLegiy = lLeg.getDimension('w')*lLeg.getDimension('w') + lLeg.getDimension('d')*lLeg.getDimension('d');
// real lLegiz = lLeg.getDimension('w')*lLeg.getDimension('w') + lLeg.getDimension('h')*lLeg.getDimension('h');

// Matrix3 lLegInertiaMatrix;
// lLegInertiaMatrix.setDiagonal(lLegix, lLegiy, lLegiz);
// lLegInertiaMatrix = lLegInertiaMatrix * (lLeg.getMass()/12);
// lLeg.setInertiaTensor(lLegInertiaMatrix);
// lLeg.name = "Left Leg";
// lLeg.setSleepEpsilon(0.02);
// lLeg.calculateDerivedData();

// cyclone::Box lLegBox;
// lLegBox.body = &lLeg;    
// lLegBox.halfSize = cyclone::Vector3(0.06, 0.44, 0.06);
// lLegBox.bindPrimitive();
// lLegBox.calculateInternals();
// lLeg.setAwake(1);

// world.addBodies(&lLeg);
// world.registry.add(&lLeg, &gravity);


// //Torso
// cyclone::Vector3 torsoPos = cyclone::Vector3(0.075,0.88+0.25,0);
// cyclone::Quaternion torsoOrient = cyclone::Quaternion(1,0,0,0);
// RigidBody torso((real)20, torsoPos, torsoOrient, (real)0.52);
// torso.setDimension(cyclone::Vector3(0.26f, 0.30f, 0.50f)); //z,x,y
// real torsoix = torso.getDimension('h')*torso.getDimension('h') + torso.getDimension('d')*torso.getDimension('d');
// real torsoiy = torso.getDimension('w')*torso.getDimension('w') + torso.getDimension('d')*torso.getDimension('d');
// real torsoiz = torso.getDimension('w')*torso.getDimension('w') + torso.getDimension('h')*torso.getDimension('h');

// Matrix3 torsoInertiaMatrix;
// torsoInertiaMatrix.setDiagonal(torsoix, torsoiy, torsoiz);
// torsoInertiaMatrix = torsoInertiaMatrix * (torso.getMass()/12);
// torso.setInertiaTensor(torsoInertiaMatrix);
// torso.name = "Torso";
// torso.calculateDerivedData();

// cyclone::Box torsoBox;
// torsoBox.body = &torso;    
// torsoBox.halfSize = cyclone::Vector3(0.15, 0.25, 0.13);

// torsoBox.offset = offset;
// torsoBox.bindPrimitive();
// torsoBox.calculateInternals();
// torso.setAwake(1);

// world.addBodies(&torso);
// world.registry.add(&torso, &gravity);


// //Head
// cyclone::Vector3 headPos = cyclone::Vector3(0.075,0.11+0.88+0.5,0);
// cyclone::Quaternion headOrient = cyclone::Quaternion(1,0,0,0);
// RigidBody head((real)3.0, headPos, headOrient, (real)0.24);
// head.setDimension(cyclone::Vector3(0.18,0.15,0.22));
// real headix = head.getDimension('h')*head.getDimension('h') + head.getDimension('d')*head.getDimension('d');
// real headiy = head.getDimension('w')*head.getDimension('w') + head.getDimension('d')*head.getDimension('d');
// real headiz = head.getDimension('w')*head.getDimension('w') + head.getDimension('h')*head.getDimension('h');

// Matrix3 headInertiaMatrix;
// headInertiaMatrix.setDiagonal(headix, headiy, headiz);
// headInertiaMatrix = headInertiaMatrix * (head.getMass()/12);
// head.setInertiaTensor(headInertiaMatrix);
// head.name = "Head";
// head.setAwake(1);
// head.calculateDerivedData();

// cyclone::Box headBox;
// headBox.body = &head;    
// headBox.halfSize = cyclone::Vector3(0.075, 0.11, 0.09);
// headBox.bindPrimitive();
// headBox.calculateInternals();

// world.addBodies(&head);
// world.registry.add(&head, &gravity);


// // //rArm
// // cyclone::Vector3 rArmPos = cyclone::Vector3(-1.51 , 5.0 ,0);
// // cyclone::Quaternion rArmOrient = cyclone::Quaternion(1,0,0,0);
// // RigidBody rArm((real)4.0, rArmPos, rArmOrient, (real)2.2);
// // rArm.setDimension(cyclone::Vector3(1,2,1)); //zxy
// // real rArmix = rArm.getDimension('h')*rArm.getDimension('h') + rArm.getDimension('d')*rArm.getDimension('d');
// // real rArmiy = rArm.getDimension('w')*rArm.getDimension('w') + rArm.getDimension('d')*rArm.getDimension('d');
// // real rArmiz = rArm.getDimension('w')*rArm.getDimension('w') + rArm.getDimension('h')*rArm.getDimension('h');

// // Matrix3 rArmInertiaMatrix;
// // rArmInertiaMatrix.setDiagonal(rArmix, rArmiy, rArmiz);
// // rArmInertiaMatrix = rArmInertiaMatrix * (rArm.getMass()/12);
// // rArm.setInertiaTensor(rArmInertiaMatrix);
// // rArm.name = "Right arm";
// // rArm.calculateDerivedData();

// // cyclone::Box rArmBox;
// // rArmBox.body = &rArm;    
// // rArmBox.halfSize = cyclone::Vector3(1, 0.5, 0.5);
// // rArmBox.bindPrimitive();
// // rArmBox.calculateInternals();

// // world.addBodies(&rArm);
// // world.registry.add(&rArm, &gravity);


// // //lArm
// // cyclone::Vector3 lArmPos = cyclone::Vector3(3.51,5.0,0);
// // cyclone::Quaternion lArmOrient = cyclone::Quaternion(1,0,0,0);
// // RigidBody lArm((real)4.0, lArmPos, lArmOrient, (real)2.2);
// // lArm.setDimension(cyclone::Vector3(1,2,1));

// // real lArmix = lArm.getDimension('h')*lArm.getDimension('h') + lArm.getDimension('d')*lArm.getDimension('d');
// // real lArmiy = lArm.getDimension('w')*lArm.getDimension('w') + lArm.getDimension('d')*lArm.getDimension('d');
// // real lArmiz = lArm.getDimension('w')*lArm.getDimension('w') + lArm.getDimension('h')*lArm.getDimension('h');

// // Matrix3 lArmInertiaMatrix;
// // lArmInertiaMatrix.setDiagonal(lArmix, lArmiy, lArmiz);
// // lArmInertiaMatrix = lArmInertiaMatrix * (lArm.getMass()/12);
// // lArm.setInertiaTensor(lArmInertiaMatrix);
// // lArm.name = "Left arm";
// // lArm.calculateDerivedData();

// // cyclone::Box lArmBox;
// // lArmBox.body = &lArm;    
// // lArmBox.halfSize = cyclone::Vector3(1, 0.5, 0.5);
// // lArmBox.bindPrimitive();
// // lArmBox.calculateInternals();

// // world.addBodies(&lArm);
// // world.registry.add(&lArm, &gravity);



// //Create Joints
// PositionJoint lLegtoTorso(&lLeg, &torso, cyclone::Vector3( 0, 0.44 ,0), cyclone::Vector3(0.08, -0.25 , 0), 0.002);

// PositionJoint rLegtoTorso(&rLeg, &torso, cyclone::Vector3( 0, 0.44 ,0), cyclone::Vector3(-0.08, -0.25 , 0), 0.002);

// // PositionJoint lArmToTorso(&lArm, &torso, cyclone::Vector3(-1, 0, 0), cyclone::Vector3(1.5, 1, 0), 0.002);

// // PositionJoint rArmToTorso(&rArm, &torso, cyclone::Vector3(1, 0, 0), cyclone::Vector3(-1.5, 1, 0), 0.002);

// PositionJoint headtoTorso(&head, &torso, cyclone::Vector3(0, -0.11, 0), cyclone::Vector3(0, 0.25, 0), 0.002);

// world.addContactGenerator(&lLegtoTorso);
// world.addContactGenerator(&rLegtoTorso);
// // world.addContactGenerator(&lArmToTorso);
// // world.addContactGenerator(&rArmToTorso);
// world.addContactGenerator(&headtoTorso);



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

//         DrawRigidBox(
//             rLeg,
//             rLegBox.halfSize,
//             RED
//         );

//         DrawRigidBox(
//             lLeg,
//             lLegBox.halfSize,
//             GREEN
//         );

//         DrawRigidBox(
//             torso,
//             torsoBox.halfSize,
//             BLUE
//         );

//         DrawRigidBox(
//             head,
//             headBox.halfSize,
//             PURPLE
//         );

//         EndMode3D();

//         DrawFPS(20, 20);
//         EndDrawing();
// }


// return 0;
// }

// /*Create Ground*/
// /*Create each body*/
// /*Create each joint*/