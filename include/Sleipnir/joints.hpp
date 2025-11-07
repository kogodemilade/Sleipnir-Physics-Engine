#pragma once;
#include "body.hpp"
#include "contacts.hpp"

/*Joints link together two rigid bodies and makes sure they do not separate.
In a ageneral physics engine there may be many different types of join, which reduce the number of relative degrees of freedom between two objects.*/

namespace cyclone{

/*Base class for all joints*/
class Joint : public ContactGenerator{
    public:
        /*Holds the 2 bodies involved in the joint*/
        RigidBody *body[2];

        /*Holds the relative location of the connection for each body, 
    given in local coordinates*/
        Vector3 position[2];


        /*Add contact*/
        // unsigned addContact(Contact *contact, unsigned limit) const;
};


/*This joint is a common position joint: each object has a
position given in body coordinates that will be kept at the
same point in the simulation*/
class PositionJoint : public Joint{
    public:

    PositionJoint(RigidBody *body[2], Vector3 positions[2], real error);
    
    /*Holds the maximum displacement at the joint before the joint
     is considered to be violated. Normally an epsilon value It can be 
     larger, in which the joint would behave like an inelastic cable joined
    the bodies at their joint locations.*/
    real error;

    /*Generates the contacts required to restore the joint if it has been violated*/
    unsigned addContact(Contact *contact, unsigned limit) const;
};


/*Keeps a body constrained in one axis */
class RotationJoint : public Joint{
    public:

    RotationJoint(RigidBody *body[2], Vector3 positions[2], real error);
    
    /*Holds the maximum displacement at the joint before the joint
     is considered to be violated. Normally an epsilon value It can be 
     larger, in which the joint would behave like an inelastic cable joined
    the bodies at their joint locations.*/
    real error;

    void checkJoint();

    /*Generates the contacts required to restore the joint if it has been violated*/
    unsigned addContact(Contact *contact, unsigned limit) const;
};

}