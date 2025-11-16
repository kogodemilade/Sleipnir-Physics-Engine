#pragma once

#include <vector>
#include <memory>
#include "body.hpp"
#include "fgen.hpp"
#include "contacts.hpp"
#include "collide_coarse.hpp"
#include "collide_fine.hpp"

/*The world reps an independent simulation of physics. 
It keeps track of a set of rigid bodies, and provides 
means to update all.*/

namespace cyclone{
class GroundContacts;
class World {
    friend GroundContacts;
    public:
    typedef std::vector<RigidBody*> RigidBodies;
    typedef std::vector<ContactGenerator*> ContactGenerators;
    typedef std::unique_ptr<Contact[]> ContactPointer;

    private:
    //Holds a list of bodies
    std::vector<RigidBody*> rigidBodies;

    //Holds head of registered bodies
    // RigidBody *firstbody;

    //Holds list of contact generators
    ContactGenerators contactgens;



    public:
    World(unsigned maxContacts, unsigned iterations);

    ~World() = default;
    //Registry for all force generators 
    ForceRegistry registry;

    ContactResolver resolver;

    CollisionDetector detector;

    CollisionData *collision_data = new CollisionData();

    //Holds list of contacts
    // ContactPointer contactStorage;
    std::vector<Contact> contacts;

    //Potential contacts
    // PotentialContact *potentialContacts = new PotentialContact[maxContacts];

    // PotentialContact *basePotContact = new PotentialContact;

    std::vector<PotentialContact*> potentialContacts;
    
    //Root of the bvh Tree for Coarse Contact resolution
    BVHNode<BoundingSphere> root;
    // BVHNode<BoundingSphere> root = BoundingSphere(Vector3(0, 0, 0), 1000); //Change, this is for debugging only. Especially  the radius
    //Maximum contacts that can be handled
    unsigned maxContacts;

    unsigned iterations;

    //Plane
    RigidBody* plane;


    /*Calls each of the registered contact generators to report their contacts. Returns the number of generated contacts*/
    unsigned generateContacts();

    /*add particles to world*/
    void addBodies(RigidBody* body, bool plane=0);

    /*add contact generators to the world*/
    void addContactGenerator(ContactGenerator* contactGen);

    /*Perform coarse collission*/
    unsigned coarseCollision();

    /*Perform fine collision*/
    unsigned fineCollision(unsigned numPotentialContacts);

    void resetCollisionData();

    /**
    * Initializes the world for a simulation frame. This clears
    * the force and torque accumulators for bodies in the
    * world. After calling this, the bodies can have their forces
    * and torques for this frame added.
    */
    void startFrame();

    void runPhysics(real duration);
};

/*A contact generator that takes an STL vector of
rigicbody pointers and collides them against the ground*/
class GroundContacts : public cyclone::ContactGenerator {
    World::RigidBodies *bodies;
    real offset;

    public:
        void init(cyclone::World::RigidBodies *bodies, real offset=0.0);

        virtual unsigned addContact(cyclone::Contact *contact, unsigned limit) const;

};
}