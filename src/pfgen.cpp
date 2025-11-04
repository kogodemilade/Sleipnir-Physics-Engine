#include "../include/Sleipnir/pfgen.hpp" //CHANGE TO #include "cyclone/pfgen.hpp"

using namespace cyclone;

void ParticleForceRegistry::updateForces(real duration){
    Registry::iterator i = registrations.begin();
    for (; i!= registrations.end(); i++){
        i->fg->updateForce(i->particle, duration);
    }
}

void ParticleForceRegistry::add(Particle* particle, ParticleForceGenerator *fg){
    ParticleForceRegistration new_pair = {particle, fg};
    registrations.push_back(new_pair);
}

void ParticleForceRegistry::remove(Particle* particle, ParticleForceGenerator *fg){
    ParticleForceRegistration pair = {particle, fg};
    //TODO
}

void ParticleForceRegistry::clear(){
    registrations.clear();
}

ParticleGravity::ParticleGravity(const Vector3 &gravity): gravity(gravity) {}

void ParticleGravity::updateForce(Particle* particle, real duration){
    //Check that we don't have infinite mass.
    if (!particle->hasFiniteMass()) return;

    //Apply the mass-scaled force to the particle.
    particle->addForce(gravity * particle->getMass());
}


ParticleDrag::ParticleDrag(real k1, real k2): k1(k1), k2(k2) {}

void ParticleDrag::updateForce(Particle* particle, real duration){
    Vector3 force;
    particle->getVelocity(&force);

    //calculate the total drag coefficient
    real dragcoeff = force.magnitude();
    dragcoeff = k1*dragcoeff + k2*dragcoeff *dragcoeff;

    //Calculate the final force and apply
    force.normalize();
    force*= -dragcoeff;
    particle->addForce(force);
}


ParticleSpring::ParticleSpring(Particle *other, real springConstant, real restLength): other(other), springConstant(springConstant), restLength(restLength) {}

void ParticleSpring::updateForce(Particle* particle, real duration){
    //calculate spring vector
    Vector3 force;
    particle->getPosition(&force);
    force -= other->getPosition();

    //Calculate the magnitude of the force.
    real magnitude = force.magnitude();
    magnitude = real_abs(magnitude - restLength);
    magnitude *= springConstant;

    //Calculate the final force and apply it
    force.normalize();
    force *= -magnitude;
    particle->addForce(force);
}


ParticleAnchoredSpring::ParticleAnchoredSpring(Vector3 *anchor, real springConstant, real restLength): anchor(anchor), springConstant(springConstant), restLength(restLength) {}

void ParticleAnchoredSpring::updateForce(Particle *particle, real duration){
    //Calculate the vector of the spring
    Vector3 force;
    particle->getPosition(&force);
    force -= *anchor;

    //calculate the magnitude of the force.
    real magnitude = force.magnitude();
    magnitude = real_abs(magnitude - restLength);
    magnitude *=springConstant;

    //Calculate the final force and apply
    force.normalize();
    force *= -magnitude;
    particle->addForce(force);

}

ParticleBungee::ParticleBungee(Particle *other, real springConstant, real restLength): other(other), springConstant(springConstant), restLength(restLength) {}


void ParticleBungee::updateForce(Particle* particle, real duration){
    //calculate the vector of the spring.
    Vector3 force;
    particle->getPosition(&force);
    force -= other->getPosition();

    //Check if the bungee is compressed.
    real magnitude = force.magnitude();
    if (magnitude <= restLength) return;

    //Calculate the magnitude of the force
    magnitude = springConstant * (restLength - magnitude);
    force.normalize();
    force *= - magnitude;
    particle->addForce(force);

}


ParticleAnchoredBungee::ParticleAnchoredBungee(Vector3 *anchor, real springConstant, real restLength): anchor(anchor), springConstant(springConstant), restLength(restLength) {}

void ParticleAnchoredBungee::updateForce(Particle* particle, real duration){
    //calculate the vector of the spring.
    Vector3 force;
    particle->getPosition(&force);
    force -= *anchor;

    //Check if the bungee is compressed.
    real magnitude = force.magnitude();
    if (magnitude <= restLength) return;

    //Calculate the magnitude of the force
    magnitude = springConstant * (restLength - magnitude);
    force.normalize();
    force *= - magnitude;
    particle->addForce(force);

}

ParticleBouyancy::ParticleBouyancy(real maxDepth, real volume, real waterHeight, real liquidDensity): maxDepth(maxDepth), volume(volume), waterHeight(waterHeight),liquidDensity(liquidDensity) {}

void ParticleBouyancy::updateForce(Particle *particle, real duration){
    //Calculate submersion depth.
    real depth = particle->getPosition().y;

    //check if we out of water
    if (depth >= waterHeight+maxDepth) return;
    Vector3 force(0,0,0);

    //check if we aty max. depth
    if (depth <= waterHeight - maxDepth) {
        force.y = liquidDensity * volume;
        particle->addForce(force);
        return;
    }

    //Partly submerged
    force.y = liquidDensity *volume *(depth - waterHeight - maxDepth) / (2*maxDepth);
    particle->addForce(force);
}

ParticleFakeSpring::ParticleFakeSpring(Vector3 *anchor, real springConstant, real damping) : anchor(anchor), springConstant(springConstant), damping(damping) {}

void ParticleFakeSpring:: updateForce(Particle *particle, real duration){
    //Check we do not have infinite mass.
    if (!particle->hasFiniteMass()) return;
    
    //Calculate the reative position of the particle to the anchor
    Vector3 position;
    particle->getPosition(&position);
    position -= *anchor;

    //Caluclate the constant and check whether they are in bounds
    real gamma = 0.5 * real_sqrt((4*springConstant) - (damping*damping));
    if (gamma=0.0f) return;
    Vector3 c = position * (damping/(2.0f*gamma)) + particle->getVelocity() * (1.0f/gamma);

    //Calculate the target position.
    Vector3 target = ((position* real_cos(gamma * duration)) +(c*real_sin(gamma*duration))) * real_exp(-0.5f *duration * damping);

    //Calculate the resulting acceleration and therefore the force
    Vector3 accel = (target - position) * (1.0f/duration*duration) - particle->getVelocity()*duration; //CHECK
    particle->addForce(accel*particle->getMass());
}