#include "../include/Sleipnir/body.hpp" //CHANGE TO #include "Sleipnir/body.hpp"
// #include "../include/cyclone/core.hpp" //CHANGE TO #icnlude "cyclone/core.hpp"
#include <assert.h>
/**Inline function that creates a transfowm matrix from a pos and orient */
using namespace cyclone;
static inline void _calculateTransformMatrix(Matrix4 &transformMatrix, const Vector3 &pos, const Quaternion &orient) {
    transformMatrix.data[0] = 1-2*orient.y*orient.y-2*orient.z*orient.z;
    transformMatrix.data[1] = 2*orient.x*orient.y - 2*orient.w*orient.z;
    transformMatrix.data[2] = 2*orient.x*orient.z + 2*orient.w*orient.y;
    transformMatrix.data[3] = pos.x;
    transformMatrix.data[4] = 2*orient.x*orient.y +  2*orient.w*orient.z;
    transformMatrix.data[5] = 1-2*orient.x*orient.x- 2*orient.z*orient.z;
    transformMatrix.data[6] = 2*orient.y*orient.z - 2*orient.w*orient.x;
    transformMatrix.data[7] = pos.y;
    transformMatrix.data[8] = 2*orient.x*orient.z - 2*orient.w*orient.y;
    transformMatrix.data[9] = 2*orient.y*orient.z + 2*orient.w*orient.x;
    transformMatrix.data[10] = 1-2*orient.x*orient.x- 2*orient.y*orient.y;
    transformMatrix.data[11] = pos.z;
}

/*Checks validity of an inverse inertia tensor*/
static inline void _checkInverseInertiaTensor(const Matrix3 &inverseInertiaTensor){

    //Todo: perform a validity check in an assert
}

/*Internal func to do an inertia tensor transform by a rotation matrix. (This was created by an automated code generator)*/
static inline void _transformInertiaTensor(Matrix3 &iitWorld, const Matrix3 &iitBody, const Matrix4 &rotmat) {
    real t4 = rotmat.data[0]*iitBody.data[0]+rotmat.data[1]*iitBody.data[3]+rotmat.data[2]*iitBody.data[6];
    real t9 = rotmat.data[0]*iitBody.data[1]+rotmat.data[1]*iitBody.data[4]+rotmat.data[2]*iitBody.data[7];
    real t14 = rotmat.data[0]*iitBody.data[2]+rotmat.data[1]*iitBody.data[5]+rotmat.data[2]*iitBody.data[8];
    real t28 = rotmat.data[4]*iitBody.data[0]+rotmat.data[5]*iitBody.data[3]+rotmat.data[6]*iitBody.data[6];
    real t33 = rotmat.data[4]*iitBody.data[1]+rotmat.data[5]*iitBody.data[4]+rotmat.data[6]*iitBody.data[7];
    real t38 = rotmat.data[4]*iitBody.data[2]+rotmat.data[5]*iitBody.data[5]+rotmat.data[6]*iitBody.data[8];
    real t52 = rotmat.data[8]*iitBody.data[0]+rotmat.data[9]*iitBody.data[3]+rotmat.data[10]*iitBody.data[6];
    real t57 = rotmat.data[8]*iitBody.data[1]+rotmat.data[9]*iitBody.data[4]+rotmat.data[10]*iitBody.data[7];
    real t62 = rotmat.data[8]*iitBody.data[2]+rotmat.data[9]*iitBody.data[5]+rotmat.data[10]*iitBody.data[8];

    iitWorld.data[0] = t4*rotmat.data[0]+t9*rotmat.data[1]+t14*rotmat.data[2];
    iitWorld.data[1] = t4*rotmat.data[4]+t9*rotmat.data[5]+t14*rotmat.data[6];
    iitWorld.data[2] = t4*rotmat.data[8]+t9*rotmat.data[9]+t14*rotmat.data[10];
    iitWorld.data[3] = t28*rotmat.data[0]+t33*rotmat.data[1]+t38*rotmat.data[2];
    iitWorld.data[4] = t28*rotmat.data[4]+t33*rotmat.data[5]+t38*rotmat.data[6];
    iitWorld.data[5] = t28*rotmat.data[8]+t33*rotmat.data[9]+t38*rotmat.data[10];
    iitWorld.data[6] = t52*rotmat.data[0]+t57*rotmat.data[1]+t62*rotmat.data[2];
    iitWorld.data[7] = t52*rotmat.data[4]+t57*rotmat.data[5]+t62*rotmat.data[6];
    iitWorld.data[8] = t52*rotmat.data[8]+t57*rotmat.data[9]+t62*rotmat.data[10];
}

RigidBody::RigidBody(){}

RigidBody::RigidBody(real _mass, Vector3 &pos, Quaternion &orient){
    if (_mass==0 || _mass==REAL_MAX)
    {inverseMass = 0.0f;}
    else{
    inverseMass = (real)1/_mass;}

    position = pos;

    orientation = orient;
    
}

RigidBody::RigidBody(real _mass, Vector3 &pos, Quaternion &orient, real _size){
    if (_mass==0 || _mass==REAL_MAX)
    {inverseMass = 0.0f;}
    else{
    inverseMass = (real)1/_mass;}

    position = pos;

    orientation = orient;

    size = _size;
}
 
bool RigidBody::hasFiniteMass(){
    if (inverseMass > 0.0) return 1;
    else return 0;
}


void RigidBody::setInertiaTensor(const Matrix3 &inertiaTensor){
    inverseInertiaTensor.setInverse(inertiaTensor);
    // _checkInverseInertiaTensor(inverseInertiaTensor);
}

void RigidBody::calculateDerivedData(){
    orientation.normalize();
    _calculateTransformMatrix(transformMatrix, position, orientation);
    //Calculate the inertiaTensor in world space.
    _transformInertiaTensor(inverseInertiaTensorWorld, inverseInertiaTensor, transformMatrix);
    state.setOrientAndPos(orientation, position);
    
}

void RigidBody::addForce(const Vector3 &force){
    setAwake(1);
    forceAccum += force;
}

void RigidBody::clearAccumulators(){
    forceAccum.clear();
    torqueAccum.clear();
}

void RigidBody::addForceAtBodyPoint(const Vector3 &force, const Vector3 &point) {
    //Convert the point from body space (relative to center of mass) to world space.
    Vector3 pt;
    pt = pt.localToWorld(point, transformMatrix);
    addForceAtPoint(force, pt);
}

void RigidBody::addForceAtPoint(const Vector3 &force, const Vector3 &point){
    setAwake(1);
    Vector3 pt = point;
    pt -= position;
    
    forceAccum += force;
    torqueAccum += pt % force;
}

void RigidBody::addTorque(const Vector3 &torque){
    setAwake(1);
    torqueAccum += torque;
}

real RigidBody::getInverseMass() const{
    return inverseMass;
}

void RigidBody::setInverseMass(const real invMass){
    inverseMass = invMass;
}

real RigidBody::getMass() const{
    if (inverseMass == (real)0.0){
        return REAL_MAX;
    }
    return (real)1.0/inverseMass;
}

void RigidBody::setLinearDamping(const real damp) {
    assert(damp > 0 && damp < 1);
    linearDamping = damp;
}

/*Get linear damoing*/
real RigidBody::getLinearDamping() const{
    return linearDamping;
}

/*Set angular damping*/
void RigidBody::setAngularDamping(const real damp){
    assert(damp > 0 && damp < 1);
    angularDamping = damp;
}

/*Get angular damping*/
real RigidBody::getAngularDamping() const{
    return angularDamping;
}

void RigidBody::setMass(const real mass){
    if (mass == 0.0){
        inverseMass = REAL_MAX;
        return;
    }
    inverseMass = (real)1.0/mass;
}

Vector3 RigidBody::getPosition() const{
    return position;
}

void RigidBody::setPosition(const Vector3 &pos){
    position = pos;
}

Quaternion RigidBody::getOrientation() const{
    return orientation;
}

void RigidBody::setOrientation(const Quaternion &orient){
    orientation = orient;
}

Vector3 RigidBody::getRotation() const{
    return rotation;
}

void RigidBody::setRotation(const Vector3 &rotation_) {
    rotation = rotation_;
}

Vector3 RigidBody::getVelocity() const{
    return velocity;
}

void RigidBody::setVelocity(const Vector3 &vel){
    velocity = vel;
}

Vector3 RigidBody::getAcceleration() const{
    return acceleration;
}

Vector3 RigidBody::getPrevAcceleration(){
    return lastFrameAcceleration;
}

void RigidBody::setAcceleration(const Vector3 &acc){
    acceleration = acc;
}

Vector3 RigidBody::getAngAcceleration() const{
    return angularAcceleration;
}

void RigidBody::setAngAcceleration(const Vector3 &angAcceleration){
    angularAcceleration = angAcceleration;
}

const Matrix3& RigidBody::getInvInertiaTensor() const{
    return inverseInertiaTensor;
}

void RigidBody::setInvInertiaTensor(const Matrix3 &invInertiaTensor){
    inverseInertiaTensor = invInertiaTensor;
}

const Matrix3& RigidBody::getInvInertiaTensorWorld() const{
    return inverseInertiaTensorWorld;
}

void RigidBody::setInvInertiaTensorWorld(const Matrix3 &invInertiaTensor){
    inverseInertiaTensorWorld = invInertiaTensor;
}

void RigidBody::updateVelocity(Vector3 &addedVelocity){
    velocity += addedVelocity; 
}

void RigidBody::updateRotation(Vector3 &addedRotation){
    rotation += addedRotation; 
}

void RigidBody::updatePosition(Vector3 &addedPos){
    position += addedPos; 
}

void RigidBody::rotateByVector(Vector3 &rot){
    orientation.rotateByVector(rot);
}

void RigidBody::rotate(Quaternion &q){
    orientation.rotate(q);
}

Vector3 RigidBody::getPointInLocalSpace(const Vector3 &pt) const{
    return transformMatrix.transformInverse(pt);
}

Vector3 RigidBody::getPointInWorldSpace(const Vector3 &pt) const{
    return transformMatrix.transform(pt);
}

Matrix4 RigidBody::getTransform() const{
    return transformMatrix;
}

// void setTransform(const Matrix4 &transformMatrix_){
//     transformM = transformMatrix_;
// }

void RigidBody::setSleepEpsilon(real eps){
    sleepEpsilon = eps;
}

real RigidBody::getSleepEpsilon() const{
    return sleepEpsilon;
}

void RigidBody::setAwake(const bool awake){
    if (awake) {
        isAwake=true;
        /*Add a bit of motion to avoid it falling asleep immediately*/
        motion = sleepEpsilon*2.0f;
    } else {
        isAwake = false;
        velocity.clear();
        rotation.clear();
    }
}

bool RigidBody::getState() const{
    return isAwake;
}

bool RigidBody::getCanSleep() const{
    return canSleep;
}

void RigidBody::setCanSleep(bool sleepable){
    canSleep = sleepable;
}

void RigidBody::checkShouldSleep(real _bias, real duration){
    /* A mass-independent approximation of total kinetic energy the body experiencex*/
    real currentMotion = velocity.scalarProduct(velocity) + rotation.scalarProduct(rotation);

    /*Recency weighted Sverage for a rolling average of the motion (or energy)*/
    // real bias = real_pow(_bias, duration);
    real bias = _bias;
    motion = bias*motion + (1-bias)*currentMotion;

    if (motion > 10*sleepEpsilon) motion = 10*sleepEpsilon;
    
    if (motion < sleepEpsilon && isAwake){
        setAwake(false);
    }
}

void RigidBody::integrate(real duration) {
    if (!isAwake) return;
    checkShouldSleep(0.6, duration);
    //Calculate linear acceleration from force inputs.
    // lastFrameAcceleration = acceleration;
    acceleration = Vector3(0, 0, 0);
    // lastFrameAcceleration.addScaledVector(forceAccum, inverseMass);
    acceleration.addScaledVector(forceAccum, inverseMass);
    lastFrameAcceleration = acceleration;

    //Calculate angular acceleration from torque inputs
    angularAcceleration = inverseInertiaTensorWorld.transform(torqueAccum);

    //Adjust velocities
    //Update linear velocity from both acceleration and impulse
    velocity.addScaledVector(acceleration, duration);

    //Update angular velocity from both acc and impulse
    rotation.addScaledVector(angularAcceleration, duration);

    //Impose drag.
    velocity *= real_pow(linearDamping, duration);
    rotation *= real_pow(angularDamping, duration);

    //Adjust positions
    //Update linear position
    position.addScaledVector(velocity, duration);

    //update angular position
    orientation.addScaledVector(rotation, duration);

    /*Normalize orientation and update matrices with new pos and orientation*/
    calculateDerivedData();

    //clear accumulators.
    clearAccumulators();
}