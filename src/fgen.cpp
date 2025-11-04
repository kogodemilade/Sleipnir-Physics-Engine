#include "../include/Sleipnir/fgen.hpp" //CHANGE TO #include "cyclone/fgen.hpp"

using namespace cyclone;

Gravity::Gravity(const Vector3 &grav): gravity(grav) {}

void Gravity::updateForce(RigidBody* body, real duration){
    //check !infinite mass
    if (!body->hasFiniteMass()) return;

    //Apply the mass-scaled force to the body
    body->addForce(gravity * body->getMass());
}

Spring::Spring(const Vector3 &localConnectionPt, RigidBody *other, const Vector3 &otherConnectionPt, real springConstant, real restLength) :
connectionPoint(localConnectionPt), 
other(other),
otherCoonectionPoint(otherConnectionPt),
springConstant(springConstant),
restLength(restLength) 
{}

void Spring::updateForce(RigidBody *body, real duration) {
    //Calculate the two ends in world space.
    Vector3 lws = body->getPointInWorldSpace(connectionPoint);
    Vector3 ows = other->getPointInWorldSpace(otherCoonectionPoint);

    //Calculate the vector of the spring
    Vector3 force = lws - ows;

    //Calculate the magnitude of the force
    real magnitude = force.magnitude();
    magnitude = real_abs(magnitude-restLength);
    magnitude *= springConstant;
    
    //Calculate the final force (normalize to get direction)
    force.normalize();
    force *= - magnitude;
    body->addForceAtPoint(force, lws);
}

void ForceRegistry::updateForces(real duration){
    // Registry::iterator i = registrations.begin();
    // for (; i!= registrations.end(); i++){
    //     i->fg->updateForce(i->particle, duration);
    // }
    for(auto reg: registrations){
        reg.fg->updateForce(reg.body, duration);
    }
}

void ForceRegistry::add(RigidBody* body, ForceGenerator *fg){
    ParticleForceRegistration new_pair = {body, fg};
    registrations.push_back(new_pair);
}