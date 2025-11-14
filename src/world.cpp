#include "../include/Sleipnir/world.hpp"

using namespace cyclone;

World::World(unsigned maxContacts, unsigned iterations):
    maxContacts(maxContacts),
    iterations(iterations),
    resolver(iterations),
    contactStorage (std::make_unique<cyclone::Contact[]>(maxContacts)),
    contacts (contactStorage.get())
{
    collision_data->contactsLeft = maxContacts;
    collision_data->contactArray = new Contact[maxContacts];
    collision_data->contacts = new Contact;
}

void World::startFrame() {
    for (auto body: rigidBodies){
        body->clearAccumulators();
        body->calculateDerivedData();
    }
}

void World::resetCollisionData(){
    for(int i=0; i<maxContacts-1; i++){
        potentialContacts[i] = PotentialContact();
        collision_data[i] = CollisionData();
    }

}

unsigned World::generateContacts() {
    unsigned limit = maxContacts;
    Contact* nextContact = contacts;

    for (auto contactGen : contactgens){
        unsigned used = contactGen->addContact(nextContact, limit);
        limit -= used;
        nextContact += used;

        if(limit <= 0) break;
    }
    return maxContacts - limit;
}

void World::addBodies(RigidBody* body){
    rigidBodies.push_back(body);
}

void World::addContactGenerator(ContactGenerator* contactgen){
    contactgens.push_back(contactgen);
}

unsigned World::coarseCollision(){
    //Coarse collision detection
    for (auto body: rigidBodies){
        root.insert(body, BoundingSphere(body->getPosition(), body->getSize()));
    }
    //Get all Potential contacts from coarse collision
    return root.getPotentialContacts(potentialContacts, 1000);
}

unsigned World::fineCollision(unsigned numPotentialContacts){
    unsigned numContacts = 0;
    for (int i=0; i<numPotentialContacts; i++){
        RigidBody *body0 = potentialContacts[i].body[0];
        RigidBody *body1 = potentialContacts[i].body[1];

        Primitive *body0_prim = body0->getPrimitive();
        Primitive *body1_prim = body1->getPrimitive();

        switch(body0_prim->getType()){
            case(PRIMITIVESPHERE):
                switch(body1_prim->getType()){
                    case(PRIMITIVESPHERE):
                        numContacts += detector.sphereAndSphere(static_cast<Sphere&>(*body0_prim), static_cast<Sphere&>(*body1_prim), collision_data);
                        break;
                    case(PRIMITIVEBOX):
                        numContacts += detector.boxAndSphere(static_cast<Box&>(*body1_prim), static_cast<Sphere&>(*body0_prim), collision_data);
                        break;
                    case(PRIMITIVEPLANE):
                        numContacts += detector.sphereAndHalfSpace(static_cast<Sphere&>(*body0_prim), static_cast<Plane&>(*body1_prim), collision_data);
                        break;
                        }
            case(PRIMITIVEBOX):
                switch(body1_prim->getType()){
                    case(PRIMITIVESPHERE):
                        numContacts += detector.boxAndSphere(static_cast<Box&>(*body0_prim), static_cast<Sphere&>(*body1_prim), collision_data);
                        break;
                    case(PRIMITIVEBOX):
                        numContacts += detector.boxAndBox(static_cast<Box&>(*body0_prim), static_cast<Box&>(*body1_prim), collision_data);
                        break;
                    case(PRIMITIVEPLANE):
                        numContacts += detector.boxAndHalfSpace(static_cast<Box&>(*body0_prim), static_cast<Plane&>(*body1_prim), collision_data);
                        break;
                        }
            case(PRIMITIVEPLANE):
                switch(body1_prim->getType()){
                    case(PRIMITIVESPHERE):
                        numContacts += detector.sphereAndHalfSpace(static_cast<Sphere&>(*body1_prim), static_cast<Plane&>(*body0_prim), collision_data);
                        break;
                    case(PRIMITIVEPLANE):
                        numContacts += detector.boxAndHalfSpace(static_cast<Box&>(*body1_prim), static_cast<Plane&>(*body0_prim), collision_data);
                        break;
                    }
                }
    }
    return numContacts;
}


void World::runPhysics(real duration){
    //Apply force gens
    registry.updateForces(duration);

    //Integrate objects
    for (auto body: rigidBodies){
        body->integrate(duration);
    }
    unsigned numPotContacts = coarseCollision();
    unsigned collisions = fineCollision(numPotContacts);


    unsigned used_contacts = generateContacts();
    resolver.resolveContacts(contacts, used_contacts, duration); //numIterations set to 1000, togglable
    resolver.resolveContacts(collision_data->contacts, collisions, duration);
    root.reset();
    // delete[] root;
    
    // resetCollisionData();
}


void GroundContacts::init(cyclone::World::RigidBodies *bodies, real offset) {
    bodies = bodies;
    offset = offset;
}

unsigned GroundContacts::addContact(cyclone::Contact *contact, unsigned limit) const {
    unsigned count = 0;
    for (cyclone::World::RigidBodies::iterator b = bodies->begin(); b!=bodies->end(); b++){
        real y = (*b)->getPosition().y;
        if (y<offset){
            contact->contactNormal = Vector3(0,1,0);
            contact->body[0] = *b;
            contact->body[1] = nullptr;
            contact->penetration = -y;
            contact->restitution = 0.2f;
            // contact->setData());
            contact++;
            count++;
        }
        if (count >= limit) return count;
    }
    return count;
}