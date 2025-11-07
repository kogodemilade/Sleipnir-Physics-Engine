#pragma once

#include "precision.hpp"
#include "core.hpp"
// #include ""

/*A rigid body is the bsic simulation object in the physics core.*/
namespace cyclone{
class RigidBody {
    protected:
        /*Holds thhe inverse mass of the rigid body. More useful than mass due to numerical stability*/
        real inverseMass;

        /*Holds the linear position of the rigid body in world space*/
        Vector3 position;

        /*Holds the angular orientation of the rigid body in world space*/
        Quaternion orientation;

        /*Holds the angular velocity, or rotation, of the rigid body in world space.*/
        Vector3 rotation;

        /*Holds the linear velocity of the rigid body in world space.*/
        Vector3 velocity;

        /*Holds the angular acceleration of the rigid body in world space*/
        Vector3 angularAcceleration;

        /*Holds the linear velocity of the rigid body in world space*/
        Vector3 acceleration;

        /*Holds the amount of damping applied to the linear motion. 
        Damping is required to remove energy added through numerical 
        instability in the integrator.*/
        real linearDamping;

        /*Holds the amount of damping applied to the angular motion.
        @see linearDamping*/
        real angularDamping;

        /*Force accumulator*/
        Vector3 forceAccum;

        /*Torque accumulator*/
        Vector3 torqueAccum;

        /*Holds a transformation matrix for converting body space into world space and 
        vice versa. This can be achieved by calling the getPointIN *Space functions.*/
        Matrix4 transformMatrix;

        /*Holds the inverse of the body's inertia tensor in local space. 
        inertia tensor provided must not be degenerate.
        The inverse tensor is used for the same reasons as for inverse mass. 
        Inertia tensor is given in body space.
        @see inverseMass*/
        Matrix3 inverseInertiaTensor;

        /*Holds the inverse of the body's inertia tensor in world coordinates
        @see inverseInertiaTensor */
        Matrix3 inverseInertiaTensorWorld;

        /*Holds the linear accleration of the body in the previous frame*/
        Vector3 lastFrameAcceleration;

        /*Holds the amount of motion of the body. This is a recency-
        weighted mean that can be used to put a body to sleep.*/
        real motion;

        /*A body can be put to sleep to avoid it being 
        updated by the integration functions or affected by 
        collisions with the world*/
        bool isAwake;

        /*Some bodies may never be allowed to fall asleep. 
        User-controlled bodies, for example, should be always awake.*/
        bool canSleep;

        /*Threshold to dheck when deciding whether a body is asleep or not*/
        real sleepEpsilon=0.2;

    public:
        /*Creates a new rigid body instance*/
        RigidBody();

        /*Creates a new rigid body instance given mass, position and orientation. Set mass=0 or REAL_MAX for infinitely heavy weights*/
        RigidBody(real mass, Vector3 &pos, Quaternion &orientation);

        /*Calculates internal data from state data.
         This should be called after the body's state is altered directly 
         (called automatically during integration). If you change the body's 
         state and then intend to integrate before querying any data (such as 
         transform matrix), then you can omit this step.*/
         void calculateDerivedData();

         /*Set Inverse Inertia tensor*/
         void setInertiaTensor(const Matrix3 &inertiaTensor);

         /*Adds the given force to the center of mass of the rigid body.
         Force is expressed in world coord.
         @param force The force to apply*/
         void addForce(const Vector3 &force);

         /*clear force and torque accumulators*/
         void clearAccumulators();

         /*Adds the given force to the given point on the rigid body.
        The direction of the force is given in world coordinates, but
        the application point is given in body space. This is useful for
        spring forces, or other forces fixed to the body.
        @param force The force to apply.
        @param point The location at which to apply the force, in body coordinates. */
        void addForceAtBodyPoint(const Vector3 &force, const Vector3 &point);

        /*Adds given force at point in world coordinates
        @param force The force to apply
        @param oint Location in world coordinates*/
        void addForceAtPoint(const Vector3 &force, const Vector3 &point);

        /*chexk whether body has finite mass*/
        bool hasFiniteMass();

        /*Adds given torque at point in world coordinates
        @param torque The torque to apply*/
        void addTorque(const Vector3 &torque);

        /*Get inverse mass of object*/
        real getInverseMass() const;

        /*Set Inverse mass of object by its inverse mass*/
        void setInverseMass(const real invMass);

        /*Get mass of object*/
        real getMass() const;

        /*Set inverse mass of object by its mass*/
        void setMass(const real mass);

        /*Get position of body*/
        Vector3 getPosition() const;

        /*Set position of body*/
        void setPosition(const Vector3 &pos);

        /*Get orientation of object as a quaternion*/
        Quaternion getOrientation() const;

        /*Set orientation of object as a quaternion*/
        void setOrientation(const Quaternion &orient);

        /*Get rotation (angular velocity) of body*/
        Vector3 getRotation() const;

        /*Set rotation (angular velocity) of body*/
        void setRotation(const Vector3 &rotation);

        /*Get velocity of body in world space*/
        Vector3 getVelocity() const;

        /*set velocity of body in world space*/
        void setVelocity(const Vector3 &vel);

        /*Update velocity of body in world space. Equivalent to: setVelocity(getvelocity()+addition)*/
        void updateVelocity(Vector3 &addedVelocity);

        /*Update rotation of body in world space. Equivalent to: setRotation(getRotation()+addition)*/
        void updateRotation(Vector3 &addedRotation);
        
        /*Rotate body by a vector.
        @param rot The vector containing the rotation*/
        void rotateByVector(Vector3 &rot);

        /*Update position of body in world space. Equivalent to: setPosition(getPosition)+addition)*/
        void updatePosition(Vector3 &addedPos);

        /*Get acceleration of body in world space*/
        Vector3 getAcceleration() const;

        /*Set acceleration of bosy in world space*/
        void setAcceleration(const Vector3 &acc);

        /*Get angular acceleration of body in world space*/
        Vector3 getAngAcceleration() const;

        /*Set angular acceleration of body in world space*/
        void setAngAcceleration(const Vector3 &angAcceleration);

        /*Get Inverse Inertia tensor*/
        const Matrix3& getInvInertiaTensor() const;

        /*Set Inverse Inertia tensor*/
        void setInvInertiaTensor(const Matrix3 &invInertiaTensor);

        /*Get Inverse Inertia tensor in world coordinates*/
        const Matrix3& getInvInertiaTensorWorld() const;

        /*Set Inverse Inertia tensor in world coordinates*/
        void setInvInertiaTensorWorld(const Matrix3 &invInertiaTensor);

        /*Get point in local space*/
        Vector3 getPointInLocalSpace(const Vector3 &point) const;

        /*Get point in world space*/
        Vector3 getPointInWorldSpace(const Vector3 &point) const;

        /*Integrate one step to the next frame*/
        void integrate(real duration);

        /*Get transform matrix*/
        Matrix4 getTransform() const;

        /*Set transform matrix*/
        void setTransform(const Matrix4 &transformMatrix);

        /*Sets sleep epsilon. Defaults to 0.2*/
        void setSleepEpsilon(real eps=0.2);

        /*Gets sleep epsilon*/
        real getSleepEpsilon() const;

        /*Get the previous acceleration*/
        Vector3 RigidBody::getPrevAcceleration();

        /*set wake states bodies*/
        void setAwake(const bool awake);

        /*Check whether an object is asleep*/
        bool getState() const;

        /*Getter method for canSleeo*/
        bool getCanSleep() const;

        /*Setter method for canSleep*/
        void setCanSleep(bool sleepable);

        /*check whether a body should be put to sleep and sleep if so*/
        void checkShouldSleep(real bias=0.8f,  real duration);

};
}