#include "../include/Sleipnir/world.hpp"
#include "../include/Sleipnir/body.hpp"
#include "../include/Sleipnir/links.hpp"
#include "../include/Sleipnir/collide_fine.hpp"
#include "../include/Sleipnir/collide_coarse.hpp"
#include "../include/Sleipnir/core.hpp"
#include "../include/Sleipnir/precision.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    cyclone::World world(1000, 1000);

    // Create mass
    cyclone::RigidBody b;
    b.setMass(5.0f);
    b.setVelocity(cyclone::Vector3(0, 0, 0));
    b.setPosition(cyclone::Vector3(200, 10, 0)); 
    b.setLinearDamping(0.99);
    b.setSize(0.5);
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
    sphere.radius = (cyclone::real)5.0;
    cyclone::Matrix4 offset;
    offset.setOrientAndPos(cyclone::Quaternion(1, 0, 0, 0), cyclone::Vector3(0, 0, 0));
    sphere.offset = offset;
    sphere.bindPrimitive();
    sphere.calculateInternals();


    //Create ground
    cyclone::RigidBody ground;
    ground.setMass(0);
    ground.setPosition(cyclone::Vector3(0,1,0));
    ground.setSize(10000.0f);
    cyclone::Vector3 groundInertiaX(0.0, 0.0, 0.0);
    cyclone::Vector3 groundInertiaY(0.0, 0.0, 0.0);
    cyclone::Vector3 groundInertiaZ(0.0, 0.0, 0.0);
    cyclone::Matrix3 groundTensor;
    groundTensor.setComponents(groundInertiaX, groundInertiaY, groundInertiaZ);
    ground.setInvInertiaTensor(groundTensor);

    cyclone::Plane groundPlane;
    groundPlane.body = &ground;
    groundPlane.normal = cyclone::Vector3(0, 1, 0);
    groundPlane.offset = (cyclone::real)0;
    groundPlane.bindPrimitive();
    groundPlane.calculateInternals();



     // Add gravity
    cyclone::Gravity gravity(cyclone::Vector3(0, -9.81f, 0));

    // Add to world
    world.addBodies(&b);
    world.addBodies(&ground);
    world.registry.add(&b, &gravity);

    float duration = 0.01;
    int i = 0;
    while (true) {
        world.startFrame();
        world.runPhysics(duration);

        if (i % 1 == 0) {
            std::cout << "iteration: " << i*duration << "secs\n";
            std::cout << "Position: x=" << b.getPosition().x
                      << " y=" << b.getPosition().y << "\n";
            std::cout << "Velocity: x=" << b.getVelocity().x
                      << " y=" << b.getVelocity().y << "\n";
            std::cout << "Acceleration: x=" << b.getAcceleration().x
                      << " y=" << b.getAcceleration().y << "\n\n";
            
            // for (auto data: anchor.getInvInertiaTensorWorld().data){
            // std::cout << "Inverse Inertia tensor world "<<data<<"\n";}
            // std::cout << "\n\n\n\n";
        }

        i++;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return 0;
}
