#pragma once

#include "core.hpp"

/*A particle (or point mass) is the simplest object that can be simulated in the physics system.*/

namespace cyclone{
class Particle{
    public:

    protected:
        /*Holds the linear position of the particle in world space.*/
        Vector3 position;

        /*Holds the linear velocity of the particle in world space.*/
        Vector3 velocity;

        /*Holds the acceleration of the particle. This value can ve used to set 
        acclereation due to gravity (its primary use) or any other constant acceleration.*/
        Vector3 acceleration;

        /*Holds the amount of damping applied to linear motion. Damping is required to remove 
        energy added through numerical instability in the integrator.*/
        real damping;

        /*Holds the inverse mass of a particle. More useful as it avoids the zero-mass problem 
        but allows for infinite mass objects. Integration is also simpler*/
        real inverseMass;

        /*Accumulates forces*/
        Vector3 forceAccum;

    public:
        /**Integrates the particle forward in time by the given amount. This function uses
         * a Newton-euler integration method, which is a linear app. of the correct integral. 
         * It may be inaccurate. Change later
         */
        void integrate(real duration);

        //Set particle's mass
        void setMass(real mass);

        //Set particle's mass
        void setInverseMass(real inverse_mass);

        //Get particle's mass
        real getMass();

        //Get Particle's Inverse mass
        real getInverseMass();

        //Set Particle's position using Vector3 vector
        void setPosition(const Vector3 &position);

        //Set particle's position using real number coordinates 
        void setPosition(const real x, const real y, const real z);

        //Get particle's position
        Vector3 getPosition();

        void getPosition(Vector3 *position) const;

        //Set particle's velocity
        void setVelocity(const Vector3 &velocity);

        //Get particle's velocity
        Vector3 getVelocity();

        void getVelocity(Vector3 *velocity) const;

        //Set particle's acceleration
        void setAcceleration(const Vector3 &acceleration);

        //Get particle's acceleration
        Vector3 getAcceleration();

        void getAcceleration(Vector3 *acceleration) const;


        //set particle's damping factor
        void setDamping(real damp_factor);

        //get particle's damping factor
        real getDamping();

        //get whether object has finite mass
        bool hasFiniteMass();

        void clearAccumulator();

        void addForce(const Vector3 &force);

};
}