#include "../include/Sleipnir/plinks.hpp" //CHANGE TO #include "cyclone/plinks.hpp"


using namespace cyclone;

real ParticleLink::currentLength() const {
    Vector3 relativePos = particle[0]->getPosition() -
                         particle[1]->getPosition();
    return relativePos.magnitude();
}

unsigned ParticleCable::addContact(ParticleContact *contact, unsigned limit) const {
    //Find cable length
    real cableLength = currentLength();

    //Check if we've overextended
    if (cableLength < maxLength) {
        return 0;
    }

    //Otherwise return the contact
    contact->particle[0] = particle[0];
    contact->particle[1] = particle[1];

    //Calculate the normal
    Vector3 normal = particle[1]->getPosition() - particle[0]->getPosition();
    normal.normalize();

    contact->contactNormal = normal;
    contact->penetration = cableLength - maxLength;
    contact->restitution = restitution;
    return 1;
}

unsigned ParticleRod::addContact(ParticleContact *contact, unsigned limit) const{
    //find the rod length
    real currentLen = currentLength();

    //Check if we're overextended
    if (fabs(currentLen - rodLength) < 1e-5f) return 0;

    //Otherwise return the contact
    contact->particle[0] = particle[0];
    contact->particle[1] = particle[1];
    
    //calculate the normal
    Vector3 normal = particle[1]->getPosition() - particle[0]->getPosition();
    normal.normalize();

    //The contact normal depends on whether we're extending or compressing
    if (currentLen > rodLength) {
        contact->contactNormal = normal;
        contact->penetration = currentLen - rodLength;
    } else {
        contact->contactNormal = normal * -1;
        contact->penetration = rodLength - currentLen;
    }

    //Always use zero restitution since we want it to be rigid
    contact->restitution = 0;

    return 1;
}
