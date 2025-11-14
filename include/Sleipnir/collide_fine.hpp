#pragma once
// #include "body.hpp"
#include "contacts.hpp"
#include "core.hpp"
/*A helper structure that contains information for 
the detector to use in building its contact data*/

namespace cyclone{
class RigidBody;
enum PrimitiveType{
    PRIMITIVESPHERE, PRIMITIVEBOX, PRIMITIVEPLANE
};

struct CollisionData {


    /*Holds the contact array to write into*/
    Contact *contactArray;

    /*Holds the first contact in the array. Makes it easier to track 
    and update contact array via pointer arithmetic*/
    Contact *contacts;

    /*Holds the maximum number of contacts the array can take*/
    unsigned contactsLeft;

    /*Holds the number of contacts used so far*/
    unsigned contactCount;

    /*Holds the restitution*/
    real restitution;

    /*Holds the lateral friction a normal*/
    real friction;

    /*Holds the tolerance for caluclating interenetration*/
    real tolerance;

    void addContacts(unsigned count);

    /*Resets data so recorded contacts are equal to 0*/
    void reset(unsigned maxContacts);

    /*Checks if there is space for more contacts*/
    bool hasContacts();

};

class Primitive {
    friend CollisionData;

    public:
        /*The rigid body represented by this primitive*/
            RigidBody *body;

        /*The offset of this primitive from the center of mass of the rigid body*/
            Matrix4 offset;

        /*Calculates internals for this primitive*/
            void calculateInternals();

        /*This is a convenience function to allow access to the axis vectors in the transform for this primitive*/
            Vector3 getAxis(unsigned index) const {
                return transform.getAxisVector(index);
            }

            virtual unsigned getType() const = 0;

        /*Bind a body to its primitive */
            void bindPrimitive();

            virtual ~Primitive() = default;
        /*Returns the resulant transform of the primitive,
        calculated from the combined offset of the primitive 
        and the transform of the rigid body to which it is attached*/
        const Matrix4& getTransform() const {
            return transform;
        }
    
    protected:
    /*The resultant of transform of the primitive. This is calculated 
    by combining the offset of the primitive with the transform of the 
    rigid body*/
    Matrix4 transform;





};


class Sphere : public Primitive {
    private:
        Vector3 pos;
    public:
        real radius;
        unsigned getType() const override {return PRIMITIVESPHERE;}

};

class Plane :public Primitive {
    public:
        Vector3 normal;

        //Distance of plane from normal
        real offset;
        unsigned getType() const override {return PRIMITIVEPLANE;}
};

class Box: public Primitive {
    public:
    /*Half of the size of the box in each axis. 
    The size in any axis would be 2 times the corresponding axis*/
        Vector3 halfSize;
        unsigned getType() const override {return PRIMITIVEBOX;}
};

class HalfPlane :public Primitive {
    public:
        Vector3 normal;

        //Distance of plane from normal
        real offset;

        //How far the plane extends
        real halfX;
        real halfY;
        unsigned getType() const override {return PRIMITIVEPLANE;}
};


class CollisionDetector {
    public:
    /*Generate contact between two spheres*/
    unsigned sphereAndSphere(Sphere &one, Sphere &two,
         CollisionData *data);

    /*Checks for contact between a sphere and half space.*/
    unsigned sphereAndHalfSpace( Sphere &sphere, 
         Plane &plane, CollisionData *data);

    /*Checks for contact between a sphere and a plane.*/
    unsigned sphereAndTruePlane(Sphere &sphere, 
        Plane &plane, CollisionData *data );

    /*Checks for contact between a box and a half space.*/
    unsigned boxAndHalfSpace( Box &box,  Plane &plane, CollisionData *data);

    /*Checks for contact between a box and a sphere.*/
    unsigned boxAndSphere( Box &box,  Sphere &sphere, CollisionData *data);

    /*Checks for contacts between two boxes.*/
    unsigned boxAndBox( Box &box1,  Box &box2, CollisionData *data);
    
    private:
    /*(Box-to-Box specific) Return how much of the box lies on given axis.*/
    real transformToAxis(const Box &box, const Vector3 &axis);

    /*(Box-to-Box specific) Returns whether the two given boxes lie overlap on the given axis, by checking 
    the distance between their centers projected onto that axis against the lengths 
    of the boxes on the axis*/
    bool overlapOnAxis(const Box &one, const Box &two, const Vector3 &axis);

    /*(Box-to-Box specific) This function checks if the two boxes overlap along the given axis, 
    returning the amount of overlap. The final parameter toCentre is used to 
    pass in the vector between the boxes centre points, to avoid having to recalculate each time.*/
    real penetrationOnAxis(const Box &one, const Box &two, const Vector3 &axis, const Vector3 &toCentre);

    /*(Box-to-Box specific) This function checks if the objects overlap on the given axis, then updates the smallestPenetration and smallestCase 
    variables to keep track of the minimum penetration (which is the one that ends up being resolved).*/
    bool tryAxis(const Box &one, const Box &two, Vector3 axis, const Vector3 &toCentre, unsigned index, real& smallestPenetration, unsigned &smallestCase);

    /*(Box-to-Box specific) Checks for overlap. Wrapper on tryAxis, and returns 0 if there's no overlap for early stop outs.*/
    unsigned checkOverlap(const Box& box1, const Box& box2, const Vector3&toCentre, Vector3 axis, unsigned index, real pen, unsigned best);


    /*(Box-to-Box specific) This method is called when we know that a vertex from box two is in contact with box one*/
    void fillPointFaceBox(const Box &box1, const Box &box2, const Vector3 &toCentre, CollisionData *data, unsigned best, real pen);

    unsigned boxAndPoint(const Box &box, const Vector3 &point, CollisionData *data);
};
}