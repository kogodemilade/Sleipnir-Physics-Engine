#include "../include/Sleipnir/pcontacts.hpp" //CHANGE TO #include "cyclone/pcontacts.hpp"

using namespace cyclone;

void ParticleContact::resolve(real duration){ 
    resolveVelocity(duration);
    resolveInterpenetration(duration);
}

real ParticleContact::calculateSeparatingVelocity() const {
    Vector3 relativeVelocity = particle[0]->getVelocity();
    if (particle[1]) relativeVelocity -= particle[1]->getVelocity();
    return relativeVelocity *contactNormal;
}

void ParticleContact::resolveVelocity(real duration){
    //Find vel in direction of contact
    real separatingVelocity = calculateSeparatingVelocity();

    //Check whther it needs to be resolved.
    if (separatingVelocity > 0){
        //Contact is either separating or stationary- no impulse required
        return;
    }

    //calculate new separating velocity
    real newSepVelocity = -separatingVelocity * restitution;

    //Check vel build-up due to acceleration only.
    Vector3 accCausedVelocity = particle[0]->getAcceleration();
    if (particle[1]) accCausedVelocity -= particle[1]->getAcceleration();
    real accCausedSepVelocity = accCausedVelocity * contactNormal * duration;

    if (accCausedSepVelocity < 0) {
        newSepVelocity -= restitution *accCausedSepVelocity;

        //Make sure we haven't removed more than was there to remove.
        if (newSepVelocity < 0) newSepVelocity = 0;
    }

    real deltavelocity = newSepVelocity - separatingVelocity;

    //we apply change in vel to each object in proportion to its inverse mass
    real totalInverseMass = particle[0]->getInverseMass();
    if (particle[1]) totalInverseMass += particle[1]->getInverseMass();

    //if all particles have inf mass, then impulses have no effect
    if (totalInverseMass <= 0) return;

    //calc impulse to apply
    real impulse = deltavelocity / totalInverseMass;

    //find the amount of impulse per unit of inverse mass
    Vector3 impulseperIMass = contactNormal * impulse;

    /*Apply impulses: they're applied in the direction of the contact, 
    and are proportional to inv mass*/
    particle[0]->setVelocity(
        particle[0]->getVelocity()+impulseperIMass * particle[0]->getInverseMass());

    if (particle[1]){
        /*Particle 1 goes in opp direction*/
        particle[1]->setVelocity(
            particle[1]->getVelocity() + impulseperIMass * -particle[1]->getInverseMass());

    }

}

void ParticleContact::resolveInterpenetration(real duration){
    /*Mass of first particle*/
    if (penetration <= 0) return;
    //Movement of each object is based on inverse mass
    real totalInverseMass = particle[0]->getInverseMass();
    if (particle[1]) totalInverseMass += particle[1]->getInverseMass();
    
    if (totalInverseMass <=0) return; //All particles have inverse mass

    //amount of penetration per inverse mass
    Vector3 movePerIMass = contactNormal * (penetration/totalInverseMass);

    //apply penetration solution
    particle[0]->setPosition(particle[0]->getPosition() + movePerIMass * particle[0]->getInverseMass() );

    if (particle[1]){
        particle[1]->setPosition(particle[1]->getPosition() - movePerIMass * particle[1]->getInverseMass() );
    }
}

ParticleContactResolver::ParticleContactResolver(unsigned iterations): iterations(iterations) {}

void ParticleContactResolver::resolveContacts(ParticleContact *contactArray, unsigned numContacts, real duration){
    iterationsUsed = 0;
    while(iterationsUsed < iterations) {
        //Find contact with largest closing velocity
        real max = REAL_MAX;
        unsigned maxIndex = numContacts;
        for (unsigned i = 0; i<numContacts; i++) {
            real sepVel = contactArray[i].calculateSeparatingVelocity();
            if (sepVel < max && (sepVel < 0 || contactArray[i].penetration > 0)) {
                max = sepVel; 
                maxIndex = i;
            }
        }
        if (maxIndex == numContacts) break;
        //Resolve this contact
        contactArray[maxIndex].resolve(duration);
        iterationsUsed++;
    }
}

void ParticleContactResolver::setIterations(unsigned it) {
    iterations = it;
}

