// Sleipnir/ragdoll.hpp
#pragma once
#include "Sleipnir/body.hpp"
#include "Sleipnir/contacts.hpp"
#include "Sleipnir/world.hpp"
#include "Sleipnir/joints.hpp"
#include <vector>
#include <memory>

namespace cyclone {
/*
 RagdollBuilder: helper that constructs a simple ragdoll using spheres and position joints.
 The function returns vectors of created bodies and joints so the caller can manage lifetime.
*/
struct Ragdoll {
    // Ownership: user is responsible for deleting these pointers later (or use smart ptrs)
    std::vector<RigidBody*> bodies;
    std::vector<ContactGenerator*> joints;
};

class RagdollBuilder {
public:
    // Create a simple ragdoll centered at `rootPos` (world space).
    // Adds created bodies to the provided World `world` and registers joints as contact generators.
    // Returns a Ragdoll struct containing the allocated bodies/joints (caller must delete when done).
    static Ragdoll createSimpleRagdoll(World& world, const Vector3& rootPos) {
        Ragdoll rag;

        // helper to allocate a spherical rigid body and register it with the world
        auto createSphereBody = [&](const Vector3& pos, real radius, real mass)->RigidBody* {
            RigidBody* rb = new RigidBody();
            if (mass <= 0) {
                rb->setInverseMass((real)0.0);
            } else {
                rb->setMass(mass);
            }

            rb->setSize(radius);
            rb->setPosition(pos);
            rb->setVelocity(Vector3(0,0,0));
            rb->setLinearDamping((real)0.98);

            // inertia approximation: I = 2/5 m r^2 but Cyclone often uses scalar * m * r^2
            real I = (real)0.4 * (mass > 0 ? mass : 1.0f) * radius * radius;
            Vector3 ix(I, I, I);
            Matrix3 inertiaMat;
            inertiaMat.setComponents(ix, ix, ix);
            rb->setInertiaTensor(inertiaMat);

            rb->calculateDerivedData();

            cyclone::Sphere sphere;
            sphere.body = rb;    
            sphere.radius = (cyclone::real)1.0;
            cyclone::Matrix4 offset;
            offset.setOrientAndPos(cyclone::Quaternion(1, 0, 0, 0), cyclone::Vector3(0, 0, 0));
            sphere.offset = offset;
            sphere.bindPrimitive();
            sphere.calculateInternals();

            world.addBodies(rb);
            rag.bodies.push_back(rb);
            return rb;
        };

        // short notation for offsets (y up)
        const real H = (real)0.4; // vertical spacing base
        Vector3 pelvisPos = rootPos;                       // pelvis center
        Vector3 torsoPos  = pelvisPos + Vector3(0, H, 0);
        Vector3 headPos   = torsoPos  + Vector3(0, H*0.9f, 0);

        Vector3 lUpperArmPos = torsoPos + Vector3(-H*0.8f, 0.0f, 0);
        Vector3 rUpperArmPos = torsoPos + Vector3( H*0.8f, 0.0f, 0);
        Vector3 lLowerArmPos = lUpperArmPos + Vector3(0, -H, 0);
        Vector3 rLowerArmPos = rUpperArmPos + Vector3(0, -H, 0);

        Vector3 lThighPos = pelvisPos + Vector3(-H*0.35f, -H, 0);
        Vector3 rThighPos = pelvisPos + Vector3( H*0.35f, -H, 0);
        Vector3 lShinPos  = lThighPos + Vector3(0, -H, 0);
        Vector3 rShinPos  = rThighPos + Vector3(0, -H, 0);

        // Create bodies (sizes and masses tuned for stability; tweak as needed)
        RigidBody* pelvis   = createSphereBody(pelvisPos,  0.18f, 12.0f);
        RigidBody* torso    = createSphereBody(torsoPos,   0.20f, 16.0f);
        RigidBody* head     = createSphereBody(headPos,    0.14f, 6.0f);

        RigidBody* lUpperArm = createSphereBody(lUpperArmPos, 0.12f, 4.0f);
        RigidBody* rUpperArm = createSphereBody(rUpperArmPos, 0.12f, 4.0f);
        RigidBody* lLowerArm = createSphereBody(lLowerArmPos, 0.11f, 3.0f);
        RigidBody* rLowerArm = createSphereBody(rLowerArmPos, 0.11f, 3.0f);

        RigidBody* lThigh = createSphereBody(lThighPos, 0.14f, 8.0f);
        RigidBody* rThigh = createSphereBody(rThighPos, 0.14f, 8.0f);
        RigidBody* lShin  = createSphereBody(lShinPos,  0.12f, 6.0f);
        RigidBody* rShin  = createSphereBody(rShinPos,  0.12f, 6.0f);

        // convenience lambda to add a PositionJoint between two bodies using world anchor points
        auto addJointByWorldPoints = [&](RigidBody* A, RigidBody* B, const Vector3& worldAnchorA, const Vector3& worldAnchorB, real error = (real)0.02) {
            // convert world anchors to local positions (if RigidBody provides getPointInLocalSpace)
            // We will assume RigidBody has a routine getPointInBodySpace or we can compute via inverse transform.
            // Here we store local positions as offsets from body positions (approximation for spheres).
            Vector3 localA = worldAnchorA - A->getPosition();
            Vector3 localB = worldAnchorB - B->getPosition();

            PositionJoint* j = new PositionJoint(A, B, localA, localB, error);
            world.addContactGenerator(j);
            rag.joints.push_back(j);
        };

        // Connect major bones (anchors use points on surface toward neighbor)
        // Pelvis <-> Torso
        addJointByWorldPoints(pelvis, torso, pelvis->getPosition() + Vector3(0,0.12f,0), torso->getPosition() + Vector3(0,-0.12f,0));

        // Torso <-> Head
        addJointByWorldPoints(torso, head, torso->getPosition() + Vector3(0,0.14f,0), head->getPosition() + Vector3(0,-0.08f,0));

        // Torso <-> upper arms
        addJointByWorldPoints(torso, lUpperArm, torso->getPosition() + Vector3(-0.18f,0.08f,0), lUpperArm->getPosition() + Vector3(0,0.08f,0));
        addJointByWorldPoints(torso, rUpperArm, torso->getPosition() + Vector3( 0.18f,0.08f,0), rUpperArm->getPosition() + Vector3(0,0.08f,0));

        // upper arm <-> lower arm
        addJointByWorldPoints(lUpperArm, lLowerArm, lUpperArm->getPosition() + Vector3(0,-0.12f,0), lLowerArm->getPosition() + Vector3(0,0.10f,0));
        addJointByWorldPoints(rUpperArm, rLowerArm, rUpperArm->getPosition() + Vector3(0,-0.12f,0), rLowerArm->getPosition() + Vector3(0,0.10f,0));

        // pelvis <-> thighs
        addJointByWorldPoints(pelvis, lThigh, pelvis->getPosition() + Vector3(-0.08f,-0.12f,0), lThigh->getPosition() + Vector3(0,0.14f,0));
        addJointByWorldPoints(pelvis, rThigh, pelvis->getPosition() + Vector3( 0.08f,-0.12f,0), rThigh->getPosition() + Vector3(0,0.14f,0));

        // thigh <-> shin
        addJointByWorldPoints(lThigh, lShin, lThigh->getPosition() + Vector3(0,-0.14f,0), lShin->getPosition() + Vector3(0,0.12f,0));
        addJointByWorldPoints(rThigh, rShin, rThigh->getPosition() + Vector3(0,-0.14f,0), rShin->getPosition() + Vector3(0,0.12f,0));

        // Done
        return rag;
    }

    // Utility: cleanup ragdoll memory (delete bodies and joints)
    static void destroyRagdoll(World& world, Ragdoll& rag) {
        // world probably doesn't provide removal helper; if it does, call it.
        for (auto* gen : rag.joints) {
            // if world keeps pointers, you might want to also remove the contact generator
            // world.removeContactGenerator(gen); // implement if you have it
            delete gen;
        }
        rag.joints.clear();

        for (auto* b : rag.bodies) {
            // world.removeBody(b); // implement if you have such API
            delete b;
        }
        rag.bodies.clear();
    }
};

} // namespace cyclone
