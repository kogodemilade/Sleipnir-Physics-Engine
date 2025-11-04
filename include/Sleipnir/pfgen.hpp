#pragma once

/**a force generator generates forces for particles */
#include "particle.hpp"
#include <vector>


namespace cyclone{
class ParticleForceGenerator{
    public:
    /*Overload this in implementations of the interfave to calculate and update the 
    force applied to the given particle.*/
    virtual void updateForce(Particle *particle, real duration) = 0;
};



/*holds all force generators and the particles they apply to*/
class ParticleForceRegistry{
    protected:
        /*Keeps track of one force gen and the particle it applies to.*/
        struct ParticleForceRegistration{
            Particle *particle;
            ParticleForceGenerator *fg;
        };

        /*holds the list of registrations*/
        typedef std::vector<ParticleForceRegistration> Registry;
        Registry registrations;

    public:
        /*Registers the given force gen to apply to the given particle*/
        void add(Particle* particle, ParticleForceGenerator *fg);

        /*Removes the given pair from registry. has no effect if pair isn't in registry*/
        void remove(Particle* particle, ParticleForceGenerator *fg);        

        /*clears all reg from registry. this doesn't delete the particles 
        themselves, just records of their connection*/
        void clear();

        /*Calls all the force gens to update the forces of their corresponding particles.*/
        void updateForces(real duration);

};



/**A force generator that applies a gravitational force. 
 * One instance can be used for multiple particles. */
class ParticleGravity : public ParticleForceGenerator{
    /*Holds the acceleration due to gravity*/
    Vector3 gravity;

    public:
    /*Creates the generator with the given acceleration*/
    ParticleGravity(const Vector3 &gravity);

    /*Applies the gravitational force to the given particle*/
    virtual void updateForce(Particle* particle, real duration);
};



//A force gen that produces a drag force
class ParticleDrag : public ParticleForceGenerator{
    /*Holds the drag coefficients. k1 is the velocity drag coefficient, 
    k2 is the velocity squared drag coefficient*/
    real k1, k2;

    public:
    /*Creates the generator with the given drag coefficients*/
    ParticleDrag(real k1, real k2);

    /*Applies the drag force to the given particle*/
    virtual void updateForce(Particle* particle, real duration);
};



/*A force gen that applies a spring force between 2 objects.*/
class ParticleSpring : public ParticleForceGenerator{
    /*Holds the spring constant*/
    real springConstant;

    /**the particle at other end of spring */
    Particle *other;

    /* holds rest length of spring*/
    real restLength;

    public:
    //creates a new spring with given params
    ParticleSpring(Particle *other, real springConstant, real restLength);

    //Applies the Spring force to the given particle
    virtual void updateForce(Particle *particle, real duration);

};



/*A force generator tha applies a spring force, where one end is attached to a fixed point in space.*/
class ParticleAnchoredSpring : public ParticleForceGenerator {
    /**The location of the anchored end of the spring. */
    Vector3 *anchor;

    /**Holds the spring constant */
    real springConstant;

    /**Holds rest length of the spring */
    real restLength;

    public:
    //creates a new spring with given params.
    ParticleAnchoredSpring(Vector3* anchor, real springConstant, real restLength);

    /*Applies spring force to given particle*/
    virtual void updateForce(Particle *particle, real duration);
};



/*A force gen that applies a spring force onlt when extended.*/
class ParticleBungee : public ParticleForceGenerator {
    /** The particle at the other end of the spring.*/
    Particle *other;

    /**Holds the spring constant */
    real springConstant;

    /**Holds the length of the bungee at the point it begins to generate a force. */
    real restLength;

    public:
    /*Creates a new bungee with the given parameters.*/
    ParticleBungee(Particle *other, real springConstant, real restLnegth);

    /**Applies the spring force to the given particle. */
    virtual void updateForce(Particle* particle, real duration);

};



/*A force gen anchored to a position that applies a spring force only when extended.*/
class ParticleAnchoredBungee : public ParticleForceGenerator {
    /** The particle at the other end of the spring.*/
    Vector3 *anchor;

    /**Holds the spring constant */
    real springConstant;

    /**Holds the length of the bungee at the point it begins to generate a force. */
    real restLength;

    public:
    /*Creates a new bungee with the given parameters.*/
    ParticleAnchoredBungee(Vector3* anchor, real springConstant, real restLnegth);

    /**Applies the spring force to the given particle. */
    virtual void updateForce(Particle* particle, real duration);

};

/*A force gen that applies a bouyancy force for a planr of liquid parallel to XZ plane*/
class ParticleBouyancy: public ParticleForceGenerator {
    /*The maximum submersion depth of the objects before it generates 
    its maximum bouyancy force.*/
    real maxDepth;

    /*The volume of the object*/
    real volume;

    /*The height of the water plane above y=0. The plane will be parallel to the XZ plane*/
    real waterHeight;

    /*The desnity of the liquid. Pure water has a density of 10^3kg/m^3*/
    real liquidDensity;

    public:
        /*creates a new bouyancy force with the given parameters*/
        ParticleBouyancy(real maxDepth, real volume, real waterHeight, real liquidDensity=1000.0);

        /*Applies the bouyancy force to the given particle*/
        virtual void updateForce(Particle *particle, real duration);
};

/* A force gen that fakes a stiff spring force and where one 
end is attached to a fixed point in space*/
class ParticleFakeSpring : public ParticleForceGenerator {
    /*The location of the anchored end of the spring*/
    Vector3 *anchor;

    /*Holds spring constant*/
    real springConstant;

    /*holds the damping on the oscillation of the spring*/
    real damping;

    public:
        /*creates a new spring force to the given particle*/
        ParticleFakeSpring(Vector3 *anchor, real springConstant, real damping);

        /*Applies the sprung force to the given particle*/
        virtual void updateForce(Particle *particle, real duration);
}; 
}