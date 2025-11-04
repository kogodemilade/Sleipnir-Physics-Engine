#include "../include/Sleipnir/particle.hpp"
#include "../include/Sleipnir/pfgen.hpp"
#include "../include/Sleipnir/pworld.hpp"
#include "../include/Sleipnir/pcontacts.hpp"
#include "../include/Sleipnir/plinks.hpp"

#include <iostream>
#include <thread>
#include <chrono>

int main() {
    cyclone::ParticleWorld world(100);

    // Create pendulum bob
    cyclone::Particle *p = new cyclone::Particle();
    p->setMass(5.0f);
    p->setVelocity(cyclone::Vector3(0, 0, 0));
    p->setPosition(cyclone::Vector3(100, 100, 0));  // displaced to side
    p->setDamping(0.99f);

    // Create fixed point (anchor)
    cyclone::Particle *anchor = new cyclone::Particle();
    anchor->setInverseMass(0.0f);  // immovable
    anchor->setPosition(cyclone::Vector3(0, 200, 0));

    // Add gravity
    cyclone::ParticleGravity gravity(cyclone::Vector3(0, -9.81f, 0));

    // Add to world
    world.addParticles(p);
    world.addParticles(anchor);
    world.registry.add(p, &gravity);

    // Connect with a rod (fixed length)
    cyclone::ParticleRod *rod = new cyclone::ParticleRod();
    rod->particle[0] = p;
    rod->particle[1] = anchor;
    rod->rodLength = 100.0f * std::sqrt(2.0f);  // same as current distance
    world.addContactGenerator(rod);
    float duration = 0.01;

    int i = 0;
    while (true) {
        world.startFrame();
        world.runPhysics(duration);

        if (i % 100 == 0) {
            std::cout << "iteration: " << i*duration << "secs\n";
            std::cout << "Position: x=" << p->getPosition().x
                      << " y=" << p->getPosition().y << "\n";
            std::cout << "Velocity: x=" << p->getVelocity().x
                      << " y=" << p->getVelocity().y << "\n";
            std::cout << "Acceleration: x=" << p->getAcceleration().x
                      << " y=" << p->getAcceleration().y << "\n\n";
        }

        i++;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return 0;
}
