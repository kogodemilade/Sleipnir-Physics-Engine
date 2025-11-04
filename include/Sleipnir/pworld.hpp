#pragma once
/*Keeps track of a set of particles, and provides the means to update them all*/
// #include "particle.hpp"
#include "pfgen.hpp"
#include "pcontacts.hpp"
#include <vector>
#include <memory>

namespace cyclone{
class GroundContacts;
class ParticleWorld{
    typedef std::vector<Particle*> Particles;
    typedef std::vector<ParticleContactGenerator*> ContactGenerators;
    typedef std::unique_ptr<ParticleContact[]> ContactPointer;

    unsigned iterations;

    friend cyclone::GroundContacts;

    public: 
        /*Holds list of particles*/
        Particles particles;

        /*Holds list of contact generators*/
        ContactGenerators contactGens;


    public:
        /*Creates a new particle simulator that can handle up to the given number
        of contacts per frame. You can also optionally give a number of contact-
        resolution iterations to use. If you don't give a number of iterations, 
        then twice the number of contacts will be used.*/
        ParticleWorld(unsigned maxContacts, unsigned iterations=0);

        /*Particle World destructor*/
        ~ParticleWorld() = default;

        /*Initializes the world for a simulation frame. This clears the force 
        accumulators for particlels in the world. After calling this, the particles
        can have their forces for this frame added.*/
        void startFrame();


        /**Holds the force gens for the particles in this world */
        ParticleForceRegistry registry;

        /*Holds the resolver for contacts*/
        ParticleContactResolver resolver;

        /*Whether to calculate how many iterations per contact or not*/
        bool calculateIterations;

        /*Holds one registered contact gen*/
        // struct ContactGenRegistration {
        //     ParticleContactGenerator *gen;
        //     ContactGenRegistration *next;
        // };

        /*Holds the list of contact generators*/
        // ContactGenRegistration *firstContactGen;     
        
        /*Holds the list of contacts*/
        ContactPointer contactStorage;
        ParticleContact* contacts;

        /*Holds the maximum number of contacts allowed (ie the size of the contacts array)*/
        unsigned maxContacts;


    public:
        /*Calls each of the registered contact generators to report their contacts. Returns the number of generated contacts*/
        unsigned generateContacts();

        /*Integrates all the particles in this world forward in time by given duration*/
        void integrate(real duration);

        /*Processes all physics for the particle world*/
        void runPhysics(real duration);

        /*returns the list of particles*/
        Particles& getParticles();

        /*returns list of contact generators*/
        ContactGenerators& getContactGens();

        /*Returns force registry*/
        ParticleForceRegistry& getRegistry();

        /*add particles to world*/
        void addParticles(Particle* particle);

        /*add contact generators to the world*/
        void addContactGenerator(ParticleContactGenerator* contactGen);

};

/*A content generator that takes an STL vector of
particle pointers and collides them against the ground*/
class GroundContacts : public cyclone::ParticleContactGenerator {
    cyclone::ParticleWorld::Particles *particles;

    public:
        void init(cyclone::ParticleWorld::Particles *particles );

        virtual unsigned addContact(cyclone::ParticleContact *contact, unsigned limit) const;


};
}