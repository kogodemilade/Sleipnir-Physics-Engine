#pragma once

#include <vector>
#include "body.hpp"
#include "fgen.hpp"

/*The world reps an independent simulation of physics. 
It keeps track of a set of rigid bodies, and provides 
means to update all.*/

namespace cyclone{
class World {
    //Holds a linked list of bodies
    std::vector<RigidBody*> rigidBodies;

    //Holds head of registered bodies
    RigidBody *firstbody;

    //Registry for all force generators 
    ForceRegistry registry;

    /**
    * Initializes the world for a simulation frame. This clears
    * the force and torque accumulators for bodies in the
    * world. After calling this, the bodies can have their forces
    * and torques for this frame added.
    */
    void startFrame();

    void runPhysics(real duration);
};
}