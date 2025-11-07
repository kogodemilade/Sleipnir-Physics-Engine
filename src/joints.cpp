#include "../include/Sleipnir/joints.hpp"

using namespace cyclone;

PositionJoint::PositionJoint(RigidBody *body_[2], Vector3 positions_[2], real error_){
    body[0] = body_[0];
    body[1] = body_[1];
    position[0] = positions_[0];
    position[1] = positions_[1];
    error = error_;
} 

unsigned PositionJoint::addContact(Contact *contact, unsigned limit) const{
    Vector3 pos0 = body[0]->getPointInWorldSpace(position[0]);
    Vector3 pos1 = body[1]->getPointInWorldSpace(position[1]);
    /*The vector that is the distance between both positions in world coordinates*/
    Vector3 distVec = pos0 - pos1;

    /*The squared magnitude of the distance vector*/
    real dist = distVec.squareMagnitude();

    /*Check if the distance between them is greater than the allowed error, if so, create contact*/
    if (dist > error*error && limit) {
        Contact *contact;

        /*Cintact point is midway between points*/
        Vector3 contactPoint =(pos0 + pos1)*0.5;

        Vector3 contactNormal = distVec.returnNormalizedVec();      
        real penetration = real_sqrt(dist);

        /*setData(const cyclone::Vector3 &contactPoint_, const cyclone::Vector3 
        &contactNormal_, cyclone::real penetration_, cyclone::RigidBody *body1_, 
        cyclone::RigidBody *body2_, cyclone::real restitution_, cyclone::real f
        riction_)*/
        contact->setData(contactPoint, contactNormal, penetration, body[0], body[1], 0, 1.0f);
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