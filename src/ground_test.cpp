// // #include "../include/Sleipnir/particle.hpp"
// // #include "../include/Sleipnir/pfgen.hpp"
// // #include "../include/Sleipnir/pworld.hpp"
// // #include "../include/Sleipnir/pcontacts.hpp"
// // #include "../include/Sleipnir/plinks.hpp"
// #include "../include/Sleipnir/world.hpp"
// #include "../include/Sleipnir/body.hpp"
// #include "../include/Sleipnir/links.hpp"
// #include "../include/Sleipnir/collide_fine.hpp"
// #include "../include/Sleipnir/collide_coarse.hpp"
// #include "../include/Sleipnir/core.hpp"
// #include "../include/Sleipnir/precision.hpp"
// #include <iostream>
// #include <thread>
// #include <chrono>

// int main() {
//     // cyclone::ParticleWorld world(1000);

//     // // Create pendulum bob
//     // cyclone::Particle *p = new cyclone::Particle();
//     // p->setMass(5.0f);
//     // p->setVelocity(cyclone::Vector3(0, 0, 0));
//     // p->setPosition(cyclone::Vector3(100, 100, 0));  // displaced to side
//     // p->setDamping(0.99f);

//     // // Create fixed point (anchor)
//     // cyclone::Particle *anchor = new cyclone::Particle();
//     // anchor->setInverseMass(0.0f);  // immovable
//     // anchor->setPosition(cyclone::Vector3(0, 200, 0));

//     // // Add gravity
//     // cyclone::ParticleGravity gravity(cyclone::Vector3(0, -9.81f, 0));

//     // // Add to world
//     // world.addParticles(p);
//     // world.addParticles(anchor);
//     // world.registry.add(p, &gravity);

//     // // Connect with a rod (fixed length)
//     // cyclone::ParticleRod *rod = new cyclone::ParticleRod();
//     // rod->particle[0] = p;
//     // rod->particle[1] = anchor;
//     // rod->rodLength = 100.0f * std::sqrt(2.0f);  // same as current distance
//     // world.addContactGenerator(rod);
//     // float duration = 0.01;

//     // int i = 0;
//     // while (true) {
//     //     world.startFrame();
//     //     world.runPhysics(duration);

//     //     if (i % 100 == 0) {
//     //         std::cout << "iteration: " << i*duration << "secs\n";
//     //         std::cout << "Position: x=" << p->getPosition().x
//     //                   << " y=" << p->getPosition().y << "\n";
//     //         std::cout << "Velocity: x=" << p->getVelocity().x
//     //                   << " y=" << p->getVelocity().y << "\n";
//     //         std::cout << "Acceleration: x=" << p->getAcceleration().x
//     //                   << " y=" << p->getAcceleration().y << "\n\n";
//     //     }

//     //     i++;
//     //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
//     // }
//     cyclone::World world(1000, 1000);

//     // Create pendulum bob
//     cyclone::RigidBody b;
//     b.setMass(5.0f);
//     b.setVelocity(cyclone::Vector3(0, 0, 0));
//     b.setPosition(cyclone::Vector3(200, 100, 0));  // displaced to side
//     b.setLinearDamping(0.99);
//     b.setSize(0.5);
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
//     anchor.setPosition(cyclone::Vector3(100, 200, 0));
//     anchor.setSize(0.1);
//     cyclone::Vector3 inertiaX_(0, 0, 0);
//     cyclone::Vector3 inertiaY_(0, 0, 0);
//     cyclone::Vector3 inertiaZ_(0, 0, 0);
//     cyclone::Matrix3 maxMatrix;
//     maxMatrix.setComponents(inertiaX_, inertiaY_, inertiaZ_);
//     anchor.setInvInertiaTensor(maxMatrix);
//     anchor.calculateDerivedData();

//     // Add gravity
//     cyclone::Gravity gravity(cyclone::Vector3(0, -9.81f, 0));

//     // Add to world
//     world.addBodies(&b);
//     world.addBodies(&anchor);
//     world.registry.add(&b, &gravity);

//     // Connect with a rod (fixed length)
//     cyclone::Rod rod;
//     rod.body[1] = &anchor;
//     rod.body[0] = &b;
//     rod.rodLength = 100.0f * std::sqrt(2.0f);  // same as current distance
//     world.addContactGenerator(&rod);

//     cyclone::Sphere sphere;
//     sphere.body = &b;    
//     sphere.radius = (cyclone::real)5.0;
//     cyclone::Matrix4 offset;
//     offset.setOrientAndPos(cyclone::Quaternion(1, 0, 0, 0), cyclone::Vector3(0, 0, 0));
//     sphere.offset = offset;
//     sphere.bindPrimitive();

//     cyclone::Sphere sphereAnchor;
//     sphereAnchor.body = &anchor;
//     sphereAnchor.radius = 0.1;
//     sphereAnchor.offset = offset;
//     sphereAnchor.bindPrimitive();

//     cyclone::GroundContacts ground;

//     float duration = 0.01;
//     int i = 0;
//     while (true) {
//         world.startFrame();
//         world.runPhysics(duration);

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
//         std::this_thread::sleep_for(std::chrono::milliseconds(10));
//     }
//     return 0;
// }
