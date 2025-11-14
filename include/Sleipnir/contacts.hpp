#pragma once
#include "core.hpp"
#include "body.hpp"
#include "collide_coarse.hpp"

/*A contact represents two bodies in contact. 
Resolving a contact removes their interpenetration.  
Contacts can be used to rep positional joints, by making 
the contact constraint keep the bodies in their correct orientation*/
namespace cyclone{
/*Forward declaration*/
class ContactResolver;


class Contact {
    /*Contact Resolver needs access to internals*/
    friend ContactResolver;
    public:

        /*Holds pos of contact in world coordinates. Initiated with contact*/
        Vector3 contactPoint;

        /*Holds the contact normal dir in world coordinates. Initiated with contact*/
        Vector3 contactNormal;

        /*Holds the transformation matrix that 
        transforms contact coordinates to world coordinates.
        Calculateed when internals are calculates*/
        Matrix3 contactToWorld;

        /*Holds the closing velocity of the contact.
        Set when the calculateInternals function is run*/
        Vector3 contactVelocity;

        /*Holds the depth of penetration at the contact point. 
        If both bodies are specified then the contact point should 
        be midway between the inter-penetrating points. . Initiated with contact*/
        real penetration;

        /*Holds the 2 bodies in contact. . Initiated with contact*/
        RigidBody *body[2];

        /*Holds the restitution coefficient (bounciness). 
        Values between 0 and 1. Initiated with contact*/
        real restitution;

        /*Holds the lateral friction coefficient at the contact. Initiated with contact*/
        real friction;

        /*Holds the desired change in velocity required to resolve the contact. 
        Calculated when internals are claculated*/
        real desiredDeltaVelocity;

        /*Holds the velocity change of the two objects when impulse is applied*/
        Vector3 velocityChange[2];

        /*Holds the rotational change of the two objects when torque is applied*/
        Vector3 rotationChange[2];

        /*Holds the world space coordinates of the contact point relative to the 
        center of each body. This is set when the calculateInternals function 
        is run.*/
        Vector3 relativeContactPosition[2];

        /*Holds the impulse. Updated when we calculate internals*/
        Vector3 impulse;

        /*Holds the angular inertia for both bodies. Updated when we fix interpenetration*/
        real angularInertia[2];

        /*Holds the linear inertia for both bodies. Updated when we fix interpentration*/
        real linearInertia[2];

        /*Holds the total inertia of both bodies involved in collision. Updated when we fix interpenetration*/
        real totalInertia;

        Vector3 linearChange[2];

        Vector3 angularChange[2];

        /*Set contact data
        @param ContactPoint point of contact in world coordinates
        @param contactNormal holds contact normal
        @param penetration
        @param body1 
        @param body2
        @param restitution
        @param friction*/
        void setData(const Vector3 &contactPoint_, const Vector3 &contactNormal_, 
            real penetration_, RigidBody* body1_, RigidBody* body2_, real restitution_, real friction_);


        /*Calculates contact velocity, relative contact positions, and contactToWorld*/
        void calculateInternals(real duration);

        /*Makes an orthonormal basis where the X axis is the contact normal, 
        and the Y axis is suggested.We assume X is normalized whem this function 
        is called. Default y to (0,1,0)*/
        void makeOrthonormalBasis(const Vector3 &x, Vector3 &y, Vector3 &z);

        /*Constructs an arbitary orthonormal basis for the contact. 
        Stored as a 3x3 matrix, where each vector is a column (transforms 
        contact space into world space). The X direction is arbitrarily 
        the contact normal, other axes are gotten from it*/
        void calculateContactBasis();

        /*Calculate The size of impulse*/
        void calcImpulse();
        
        /*Calculates the contact velocity and updates the contactVelocity member*/
        void calcContactVelocity(real duration);

        /*Calculates the desired change in velocity and updates the desiredDeltaVelocity member*/
        void calcDesiredDeltaVelocity(real duration);

        /*Applies the change in angular and linear velocities.*/
        void applyImpulse();

        /*Calculates the relative positions of the contact point 
        to each body and updates the relativeContactPosition data member*/
        void calcRelContactPosition();

        /*Resolves interpenetration using the nonlinear projection method*/
        void fixInterpenetration();

        /*Calculates the matrix that converts from contact to world coordinates*/
        void calcContactToWorld();

        void matchAwakeState();
};

/*The contact resolution routine. One resolver instance can be shared for the whole simulation*/
class ContactResolver {
    real penetrationEpsilon = 0.01;
    real velocityEpsilon = 0.01;
    unsigned iterations = 1000;
    public:

    ContactResolver(unsigned iterations) : iterations(iterations) {}
    /*Resolves a set of contacts for both penetration and velocity.*/
    void resolveContacts(Contact *contactArray, unsigned numContacts, real duration, unsigned numPosIterations=1000, unsigned numVelocityIterations=1000);

    protected:
    /*Sets up contacts ready for processing by calculating its internal data*/
    void prepareContacts(Contact *contactArray, unsigned numContacts, real duration);

    /*Fixes interpenetration*/
    void adjustPositions(Contact *contacts, unsigned numContacts, real duration, unsigned numPosIterations);

    /*Adjusts the velocities of colliding objects*/
    void adjustVelocities(Contact *contacts, unsigned numContacts, real duration, unsigned numVelocityPositions);

    /*Updates penetration values after a contact has been resolved*/
    void updatePenetration(Contact *contacts, unsigned numContacts, real duration, unsigned numVelocityPositions);
};

/*This is the basic polymorphic interface for contact generators applying to rigid bodies*/
class ContactGenerator {
    public:
    /*Fills the given contact structure with the generated contact. 
    The contact pointer should point to the first available contact in a 
    contact arrat, where limit is the maximum number of contacts in the array 
    that can be written to. The method returns the number of contacts that have 
    been written*/
    virtual unsigned addContact(Contact *contact, unsigned limit) const =0;
};
}