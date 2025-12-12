#include "../include/Sleipnir/joints.hpp"

using namespace cyclone;

unsigned PositionJoint::addContact(Contact *contact, unsigned limit) const{
    // Vector3 position0_ = Vector3(positions[0].x, positions[0].y, positions[0].z);
    Vector3 pos0 = body[0]->getPointInWorldSpace(positions[0]);
    Vector3 pos1 = body[1]->getPointInWorldSpace(positions[1]);
    /*The vector that is the distance between both positions in world coordinates*/
    Vector3 distVec = pos1 - pos0;

    /*The squared magnitude of the distance vector*/
    real dist = fabs(distVec.magnitude());

    /*Check if the distance between them is greater than the allowed error, if so, create contact*/
    if (dist > error && limit) {
        /*Contact point is midway between points*/
        Vector3 contactPoint =(pos0 + pos1)*0.5;

        Vector3 contactNormal = distVec.returnNormalizedVec();      
        real penetration = real_sqrt(dist) - error;

        if (distVec.magnitude() - error > 0) contactNormal *= -1; //reversing for pulling vs pushing. continue tomorrow.

        /*setData(const cyclone::Vector3 &contactPoint_, const cyclone::Vector3 
        &contactNormal_, cyclone::real penetration_, cyclone::RigidBody *body1_, 
        cyclone::RigidBody *body2_, cyclone::real restitution_, cyclone::real f
        riction_)*/
        contact->setData(contactPoint, contactNormal, penetration, body[0], body[1], 0.0, 0.0f);
        return 1;
    } else return 0;
}

// RotationJoint::RotationJoint(RigidBody *body_[2], Vector3 positions_[2], real error_){
//     body[0] = body_[0];
//     body[1] = body_[1];
//     position[0] = positions_[0];
//     position[1] = positions_[1];
//     error = error_;
// } 