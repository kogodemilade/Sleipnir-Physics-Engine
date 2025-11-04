#include "../include/Sleipnir/pworld.hpp" //CHANGE TO #include "cyclone/pworld.hpp"
#include <memory>
using namespace cyclone;

ParticleWorld::ParticleWorld(unsigned maxContacts, unsigned iterations):
    maxContacts(maxContacts),
    iterations(iterations),
    resolver(iterations),
    contactStorage (std::make_unique<cyclone::ParticleContact[]>(maxContacts)),
    contacts (contactStorage.get())
{}

ParticleWorld::Particles& ParticleWorld::getParticles(){
    return particles;
}

ParticleWorld::ContactGenerators& ParticleWorld::getContactGens() {
    return contactGens;
}

ParticleForceRegistry& ParticleWorld::getRegistry() {
    return registry;
}

void ParticleWorld::startFrame() {
    for(auto p: particles) {
        p->clearAccumulator();
    }
}

unsigned ParticleWorld::generateContacts(){
    unsigned limit = maxContacts;
    ParticleContact *nextContact = contacts;

    for(auto contactGen: contactGens) {
        // if (nextContact->particle[1]){}
        unsigned used = contactGen->addContact(nextContact, limit);
        limit -= used;
        nextContact += used;

        //We've run out of contacts to fill, meaning we're missing contacts
        if (limit <= 0) break;
    }
    //return the number of contacts used.
    return maxContacts - limit;
}

void ParticleWorld::addParticles(Particle* particle){
    ParticleWorld::particles.push_back(particle);
}

void ParticleWorld::addContactGenerator(ParticleContactGenerator *contactGen) {
    ParticleWorld::contactGens.push_back(contactGen);
}


void ParticleWorld::integrate(real duration) {
    for(auto p: particles) {
        p->integrate(duration);
    }
}

void ParticleWorld::runPhysics(real duration) {
    //First apply the force gens
    registry.updateForces(duration);

    //Then integrate the objects
    integrate(duration);

    //Generate contacts
    unsigned usedContacts = generateContacts();

    //And process them
    if (!iterations) resolver.setIterations(usedContacts*2);
    resolver.resolveContacts(contacts, usedContacts, duration);
}

void GroundContacts::init(cyclone::ParticleWorld::Particles *particles) {
    GroundContacts::particles = particles;
}

unsigned GroundContacts::addContact(cyclone::ParticleContact *contact, unsigned limit) const {
    unsigned count = 0;
    for (cyclone::ParticleWorld::Particles::iterator p = particles->begin(); p!=particles->end(); p++){
        real y = (*p)->getPosition().y;
        if (y<0.0f){
            contact->contactNormal = Vector3(0,1,0);
            contact->particle[0] = *p;
            contact->particle[1] = nullptr;
            contact->penetration = -y;
            contact->restitution = 0.2f;
            contact++;
            count++;
        }
        if (count >= limit) return count;
    }
    return count;
}