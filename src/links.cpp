#include "../include/Sleipnir/links.hpp" //CHANGE TO #include "cyclone/plinks.hpp"


using namespace cyclone;

real Link::currentLength() const {
    Vector3 relativePos = body[0]->getPosition() -
                         body[1]->getPosition();
    return relativePos.magnitude();
}

unsigned Cable::addContact(Contact *contact, unsigned limit) const {
    //Find cable length
    real cableLength = currentLength();

    //Check if we've overextended
    if (cableLength < maxLength) {
        return 0;
    }

    //Otherwise return the contact
    //Calculate the normal
    Vector3 normal = body[1]->getPosition() - body[0]->getPosition();
    normal.normalize();
    real penetration = cableLength - maxLength;
    
    contact->setData(body[0]->getPosition(), normal, penetration, body[0], body[1], restitution, 1.0);
    return 1;
}

unsigned Rod::addContact(Contact *contact, unsigned limit) const{
    //find the rod length
    real currentLen = currentLength();

    //Check if we're overextended
    if (fabs(currentLen - rodLength) < 1e-5f) return 0;

    //Otherwise return the contact
    
    //calculate the normal
    Vector3 normal = body[1]->getPosition() - body[0]->getPosition();
    normal.normalize();
    Vector3 contactNormal;
    real penetration;
    //The contact normal depends on whether we're extending or compressing
    if (currentLen > rodLength) {
        contactNormal = normal;
        penetration = currentLen - rodLength;
    } else {
        contactNormal = normal * -1;
        penetration = rodLength - currentLen;
    }

    //Always use zero restitution since we want it to be rigid
    real restitution = 0;

    // Vector3 contactPoint = (body[0]->getPosition() + body[1]->getPosition()) * 0.5;

    contact->setData(body[0]->getPosition(), normal, penetration, body[0], body[1], restitution, 0.0);
    return 1;
}
