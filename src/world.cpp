#include "../include/Sleipnir/world.hpp"

using namespace cyclone;

World::World(unsigned maxContacts, unsigned iterations):
    maxContacts(maxContacts),
    iterations(iterations),
    resolver(iterations)
    // contactStorage (std::make_unique<cyclone::Contact[]>(maxContacts))
{
    collision_data->contactsLeft = maxContacts;
    collision_data->contacts.reserve(maxContacts);
    // potentialContacts.assign()
}

void World::startFrame() {
    for (auto body: rigidBodies){
        body->clearAccumulators();
        body->calculateDerivedData();
    }
}

void World::resetCollisionData(){
    root.reset();
    collision_data->reset(maxContacts);
    // for(int i=0; i<maxContacts-1; i++){
    //     potentialContacts.pop_back();
    //     // collision_data.pop_back();
    // }
    potentialContacts.clear();
    contacts.clear();

}

unsigned World::generateContacts() {
    unsigned limit = maxContacts;

    for (auto contactGen : contactgens){
        Contact nextContact;
        // if(!&contacts[nextContact]) break;
        unsigned used = contactGen->addContact(&nextContact, limit);
        contacts.push_back(nextContact);
        limit -= used;
        // nextContact += used;

        if(limit <= 0) break;
    }
    return maxContacts - limit;
}

void World::addBodies(RigidBody* body, bool plane_){
    if (plane_){
        plane = body;
        return;
    }
    rigidBodies.push_back(body);
}

void World::addContactGenerator(ContactGenerator* contactgen){
    contactgens.push_back(contactgen);
}

unsigned World::coarseCollision(){
    //Coarse collision detection
    for (auto body: rigidBodies){
        root.insert(body, BoundingSphere(body->getPosition(), body->getSize()), 1);
    }
    //Get all Potential contacts from coarse collision
    return root.getPotentialContacts(potentialContacts, 1000);
}

unsigned World::fineCollision(unsigned numPotentialContacts){
    unsigned numContacts = 0;
    for (auto contact: potentialContacts){
        RigidBody *body0 = contact->body[0];
        RigidBody *body1 = contact->body[1];

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
                        numContacts += detector.sphereAndTruePlane(static_cast<Sphere&>(*body0_prim), static_cast<Plane&>(*body1_prim), collision_data);
                        break;
                        } break;
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
                        } break;
            case(PRIMITIVEPLANE):
                switch(body1_prim->getType()){
                    case(PRIMITIVESPHERE):
                        numContacts += detector.sphereAndHalfSpace(static_cast<Sphere&>(*body1_prim), static_cast<Plane&>(*body0_prim), collision_data);
                        break;
                    case(PRIMITIVEPLANE):
                        numContacts += detector.boxAndHalfSpace(static_cast<Box&>(*body1_prim), static_cast<Plane&>(*body0_prim), collision_data);
                        break;
                    } break;
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
        
    int i = 0;
    //collide all objects with plane
    if(plane){
    for (auto body: rigidBodies){
        if (body->getPrimitive()->getType() == PRIMITIVEPLANE) continue;
        PotentialContact *pc = new PotentialContact;
        pc->body[0] = body;
        pc->body[1] = plane;
        potentialContacts.push_back(pc);
        numPotContacts += 1;
    }}

    unsigned collisions = fineCollision(numPotContacts);

    unsigned used_contacts = generateContacts();
    resolver.resolveContacts(contacts, used_contacts, duration); //numIterations set to 1000, togglable
    resolver.resolveContacts(collision_data->contacts, collisions, duration);

    // delete[] root;
    
    resetCollisionData();
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