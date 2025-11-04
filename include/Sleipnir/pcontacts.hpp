#pragma once

/*A contact represents two objects in contact (in this case ParticleContact representing 2 particles). 
Resolving a contact removes their interpenetration, and applies sufficient impulse to keep them apart. 
Colliding bodies may also rebound.
Thee contact has no callable functions, it just holds the contact details. 
To resolve a set of contacts, use the particle contact resolver class. */
#include "particle.hpp"
#include "precision.hpp"

namespace cyclone{
class ParticleContact {

    /*Contact resolver needs access into this class to access and change variables.*/
    friend class ParticleContactResolver;


    public:
        /*Holds the particles that are involved in the contact. 
        The second of these can be NULL, for contacts with the scenery*/
        Particle* particle[2];

        /*Holds the normal restitution coefficient at the contact. */
        real restitution;

        /*Holds the direction of the contact in world coordinates*/
        Vector3 contactNormal;

        /*Holds depth of penetration at contact*/
        real penetration;

    protected:
        /*Resolves this contact, for both velocity and interpenetration*/
        void resolve(real duration);

        /*Calculates the separating velocity at this contact*/
        real calculateSeparatingVelocity() const;

    private:
        /*Handles the impulse calculations for this collision.*/
        void resolveVelocity(real duration);

        /*Handles interpenetration resolution for this contact*/
        void resolveInterpenetration(real duration);
};


class ParticleContactResolver {
    protected:
        /*Holds max number of iterations allowed*/
        unsigned iterations;

        /*Performance tracking value: we keep number of actual number of iterations used*/
        unsigned iterationsUsed;

    public:
        /*Creates a new contact resolver*/
        ParticleContactResolver(unsigned iterations);
        

        /*Sets the number of iterations that can be used*/
        void setIterations(unsigned iterations);

        /*resolves a set of particle contacts for both penetration and velocity */
        void resolveContacts(ParticleContact *contactArray, unsigned numContacts, real duration);
        

};

/*Basic Polymporphic interface for contact generators applying to particles.*/
class ParticleContactGenerator {
    public:
        /*Fills the given contact structure with the generated contact. The contact
        pointer should point to the first available contact in a contact array, where 
        limit is the maximum number of arrays that can be written to. The method returns
        the number of contacts that have been written*/
        virtual unsigned addContact(ParticleContact *contact, unsigned limit) const = 0;
};
}