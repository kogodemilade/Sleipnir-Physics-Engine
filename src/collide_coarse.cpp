#include "../include/Sleipnir/collide_coarse.hpp"

using namespace cyclone;
bool BoundingSphere::overlaps(const BoundingSphere *other) const {
    real distanceSquared = (center - other->center).squareMagnitude();
    return distanceSquared < (radius +other->radius) * (radius+other->radius);
}

BoundingSphere::BoundingSphere(const Vector3 &center, real radius): center(center), radius(radius) {}

BoundingSphere::BoundingSphere(const BoundingSphere &one, const BoundingSphere &two){
    center = (one.center + two.center)*0.5;
    radius = one.radius + two.radius + center.magnitude();
}

real BoundingSphere::getSize() const{
    return (real)2*radius;
}

real BoundingSphere::getGrowth(const BoundingSphere &newSphere) const{
    BoundingSphere newBS = BoundingSphere(*this, newSphere);
    return newBS.getSize() - (*this).getSize();
}

// void BoundingSphere::recalculateBoundingVolume(BoundingSphere *sibling){
//     center = 
// }