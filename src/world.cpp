#include "../include/Sleipnir/world.hpp"

using namespace cyclone;
void World::startFrame() {
    for (auto body: rigidBodies){
        body->clearAccumulators();
        body->calculateDerivedData();
    }
}

void World::runPhysics(real duration){
    //Apply force gens
    registry.updateForces(duration);

    //Integrate objects
    for (auto body: rigidBodies){
        body->integrate(duration);
    }
}