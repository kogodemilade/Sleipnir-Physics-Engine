#include <assert.h>
#include "../include/Sleipnir/particle.hpp" //CHANGE TO #include "cyclone/particle.hpp"
// #include "../include/cyclone/core.hpp" //CHANGE TO #include "cyclone/core.hpp"


using namespace cyclone;

void Particle::integrate(real duration){
    assert(duration>0.0);

    //Update linear position.
    position.addScaledVector(velocity, duration);

    /*ascertain acc from force*/
    // Vector3 resultingAcc = acceleration;
    acceleration = forceAccum*getInverseMass();

    //update linear vel. from the acc.
    velocity.addScaledVector(acceleration, duration);

    //impose drag.
    velocity *= real_pow(damping, duration);

    //clear forces
    forceAccum.clear();
}


void Particle::setMass(real mass){
    assert (mass != 0);
    Particle::inverseMass = ((real)1.0)/mass;
}

void Particle::setInverseMass(const real inverse_mass){
    Particle::inverseMass = inverse_mass;
}

real Particle::getMass(){
    if (inverseMass == 0){
        return REAL_MAX;
    } else{
        return ((real)1)/inverseMass;
    }
}

real Particle::getInverseMass(){
    return inverseMass;
}

void Particle::setPosition(const Vector3 &position){
    Particle::position = position;
}

void Particle::setPosition(const real x, const real y, const real z){
    position.x = x;
    position.y = y;
    position.z = z;
}

void Particle::getPosition(Vector3 *position) const {
    *position = Particle::position;
}

Vector3 Particle::getPosition(){
    return position;
}

void Particle::setVelocity(const Vector3 &velocity){
    Particle::velocity = velocity;
}

Vector3 Particle::getVelocity(){
    return velocity;
}

void Particle::getVelocity(Vector3 *velocity) const {
    *velocity = Particle::velocity;
}

void Particle::setAcceleration(const Vector3 &acceleration){
    Particle::acceleration = acceleration;
}

Vector3 Particle::getAcceleration(){
    return Particle::acceleration;
}

void Particle::getAcceleration(Vector3 *acceleration) const {
    *acceleration = Particle::acceleration;
}

void Particle::setDamping(real damping_factor){
    Particle::damping = damping_factor;
}

real Particle::getDamping(){
    return damping;
}

bool Particle::hasFiniteMass(){
    return inverseMass >=0.0f;
}

void Particle::clearAccumulator(){
    forceAccum.clear();
}

void Particle::addForce(const Vector3 &force){
    forceAccum += force;
}


