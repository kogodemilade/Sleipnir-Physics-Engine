#pragma once

#include "body.hpp"
#include <vector>
 namespace cyclone{
 class ForceGenerator{
    public:
    /*Overload this in implementations of the interfave to calculate and update the 
    force applied to the given rigid body.*/
    virtual void updateForce(RigidBody *body, real duration) = 0;
};

/*holds all force generators and the particles they apply to*/
class ForceRegistry{
    public:
        /*Keeps track of one force gen and the particle it applies to.*/
        struct ParticleForceRegistration{
            RigidBody *body;
            ForceGenerator *fg;
        };

        /*holds the list of registrations*/
        typedef std::vector<ParticleForceRegistration> Registry;
        Registry registrations;

    public:
        /*Registers the given force gen to apply to the given particle*/
        void add(RigidBody* body, ForceGenerator *fg);

        /*Removes the given pair from registry. has no effect if pair isn't in registry*/
        void remove(RigidBody* body, ForceGenerator *fg);        

        /*clears all reg from registry. this doesn't delete the particles 
        themselves, just records of their connection*/
        void clear();

        /*Calls all the force gens to update the forces of their corresponding particles.*/
        void updateForces(real duration);

};

/*A force generator that applies a gravitational force.
 One instance can be used for multiple rigid bodies.*/
 class Gravity : public ForceGenerator {
    /*Holds acceleration due to gravity*/
    Vector3 gravity;

    public:
        /*Creates the generator with the given acceleration*/
        Gravity(const Vector3 &gravity);

        /*Applies the gravitational force to the given rigid body.*/
        virtual void updateForce(RigidBody *body, real duration);
 };


/*A force generator that applies a spring force*/
class Spring : public ForceGenerator {
    /*The point of connection of the spring, in local coordinates*/
    Vector3 connectionPoint;

    /*The point of connection of the spring to the other object, in that object's local coordinates.*/
    Vector3 otherCoonectionPoint;

    /*The particle at the other end of the spring*/
    RigidBody *other;

    /*Holds the spring constant*/
    real springConstant;

    /*Holds the rest length of the spring*/
    real restLength;

    public:
        /*creates a new spring with given parameters*/
        Spring(const Vector3 &localConnectionPt, RigidBody *other, const Vector3 &otherConnectionPt, real springConstant, real restLength);

        /*Applies the spring force to the given particle*/
        virtual void updateForce(RigidBody *body, real duration);
};

/*A force gen that applies an aerodynamic force */
class Aero : public ForceGenerator {
    /*Holds the aerodynamic tensor for the surface in body space*/
    Matrix3 tensor;

    /*Holds the relative pos of the aerodynamic surface in body coordinates*/
    Vector3 pos;

    /*Holds a pointer to a vector containing the wind speed of the environment.
    Thid is easier than managing a separate wind speed vector per generator and having to update it manually as the wind changes.*/
    const Vector3* windspeed;

    public:
        /*creates a new aerodynamic force generator with the given properties*/
        Aero(const Matrix3 &tensor, const Vector3 &position, const Vector3 *windspeed);

        /*Applies the force to the given rigid body.*/
        virtual void updateForces(RigidBody *body, real duration);
};

/*A force gen with a control aerodynamic surface. This requires three inertia
tensors, for the two extremes and the 'restinf* position of the control surface.
The latter tensor is the one inherited from the base class; the 2 extremes are 
defined in this class.*/
  class AeroControl : public Aero {
    /*The aerodynamic tensor for the surface, when the control is at its max value*/
    Matrix3 maxTensor;

    /*Aerodynamic tensor for the surface, when control is at its minimum value*/
    Matrix3 MinTensor;

    /*The current position of the control for this surface. This should range
    between -1 (in which case minTensor value is used) through 0 (where base-class
    tensor val is used) to +1 (where maxTensor is used)*/
    real controlSetting;

    private: 
    /*Calcs the final aerodynamic tensor for the current control setting.*/
    Matrix3 getTensor();

    public: /*Creates a new aerodynamic control surface with the gien properties*/
    AeroControl(const Matrix3 &base, const Matrix3 &min, const Matrix3 &max,
         const Vector3 &position, const Vector3 *windspeed);

    /*Sets the control position of this control. This should 
    range between -1 and +1. Values outside that range give undefined resilts.
    @see ControlSetting*/
    void setControl(real Value);

    /*Applies the force to the given rigid body.*/
    virtual void updateForce(RigidBody *body, real duration);
  };

/*A force gen that applies a bouyancy force for a planr of liquid parallel to XZ plane*/
class Bouyancy: public ForceGenerator {
    /*The maximum submersion depth of the objects before it generates 
    its maximum bouyancy force.*/
    real maxDepth;

    /*The volume of the object*/
    real volume;

    /*The height of the water plane above y=0. The plane will be parallel to the XZ plane*/
    real waterHeight;

    /*The desnity of the liquid. Pure water has a density of 10^3kg/m^3*/
    real liquidDensity;

    /*The center of bouyancy of the rigif body, in the body coordinates.*/
    Vector3 centerOfBouyancy;

    public:
        /*creates a new bouyancy force with the given parameters*/
        Bouyancy(const Vector3 &cOfB, real maxDepth, real volume, real waterHeight, real liquidDensity=1000.0f);

        /*Applies the bouyancy force to the given particle*/
        virtual void updateForce(RigidBody *body, real duration);
};


/*A force generator with an aerodynamic surface that can be re-oriented relative to its rigid body.*/
class AngledAero: public Aero {
    /*Holds orientation of the aerodynamic surface relative to the rigid body to which it is attached*/
    Quaternion orientation;

    public:
    /*Creates a new aerodynamic syrface with given properties.*/
    AngledAero(const Matrix3 &tensor, const Vector3 &position, const Vector3 *windspeed);

    /*sets relative orientation of the aerdynamic surface relative to the 
    rigid body it's attached to. Note that this doesn't affect the point of 
    connection of the surface of the body.*/
    void setOrientation(const Quaternion &quat);

    /*Applies the force to the given rigid body*/
    virtual void updateForce(RigidBody *body, real duration);
};
 }