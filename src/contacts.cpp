#include "../include/Sleipnir/contacts.hpp"

using namespace cyclone;
void Contact::setData(const Vector3 &contactPoint_, const Vector3 &contactNormal_, 
    real penetration_, RigidBody* body1_, RigidBody* body2_, real restitution_, real friction_) {
        contactPoint = contactPoint_;
        contactNormal = contactNormal_;
        penetration = penetration_;
        body[0] = body1_;
        body[1] = body2_;
        restitution = restitution_;
        friction = friction_;
    }

void Contact::makeOrthonormalBasis(const Vector3 &x, Vector3 &y, Vector3 &z){//Default y to (0,1,0)

    /*Scaling factor to ensure the results are normalized.*/
    const real s = 1.0/real_sqrt(x.z*x.z + x.x*x.x);

    //The new Z axis is at right agles to the world Y axis.
    z.x = x.z*s;
    z.y = 0;
    z.z = -x.x*s;

    //The new Y axis is at right angles to the new X and Z axes
    y.x = x.y*z.x;
    y.y = x.z*z.x - x.x*z.z;
    y.z = -x.y*z.x;
}


void Contact::calculateContactBasis(){
    Vector3 contactTangent[2];

    /*Check whether the Z axis is nearer to the x or y axis*/
    if (real_abs(contactNormal.x) > real_abs(contactNormal.y)){
        /*Scaling factor to ensure the results are normalized*/
        const real s = (real)1.0f/ real_sqrt(contactNormal.z*contactNormal.z + contactNormal.x*contactNormal.x);

        /*The new X axis is at right angles to the world Y axis*/
        contactTangent[0].x = contactNormal.z*s;
        contactTangent[0].y = 0;
        contactTangent[0].z = -contactNormal.x*s;

        /*The new Y axis is at right angles to the new X and Z axes.*/
        contactTangent[1].x=contactNormal.y*contactTangent[0].x;
        contactTangent[1].y = contactNormal.z*contactTangent[0].x - contactNormal.x*contactTangent[0].z;
        contactTangent[1].z = -contactNormal.y*contactTangent[0].x;
    } else {
        /*Scaling factor to ensure the results are normalized*/
        const real s = (real)1.0/ real_sqrt(contactNormal.z*contactNormal.z + contactNormal.y*contactNormal.y);

        /*The new x axis is at right angles to the world x axis.*/
        contactTangent[0].x = 0;
        contactTangent[0].y = -contactNormal.z*s;
        contactTangent[0].z = contactNormal.y*s;

        /*The new y axis is at right angles to the new x and z axes*/
        contactTangent[1].x=contactNormal.y*contactTangent[0].z  - contactNormal.z*contactTangent[0].y;
        contactTangent[1].y = -contactNormal.x*contactTangent[0].z;
        contactTangent[1].z = -contactNormal.x*contactTangent[0].y;
    }

    //Create the matrix from these vectors
    contactToWorld.setComponents(contactNormal, contactTangent[0], contactTangent[1]);
}



void Contact::calcContactVelocity(){
    Vector3 deltaVelocityVec = body[0]->getRotation() % relativeContactPosition[0];
    deltaVelocityVec += body[0]->getVelocity();

    if(body[1]) {
        deltaVelocityVec += body[1]->getRotation() % relativeContactPosition[1];
        deltaVelocityVec += body[1]->getVelocity();
    }
    contactVelocity = contactToWorld.transformTranspose(deltaVelocityVec);
}

void Contact::calcDesiredDeltaVelocity(real duration){
    const static real velLowerLimit = (real)0.25f;

    //Get velocity from acceleration, V = u + at
    real velocityFromAcc = 0;
    velocityFromAcc += body[0]->getPrevAcceleration() * duration *contactNormal;
    
    if (body[1]) {
        velocityFromAcc += body[1]->getPrevAcceleration() * duration * contactNormal;
    }

    //If velocity is very low, limit restitution
    real thisRestitution = restitution;
    if (real_abs(contactVelocity.x) < velLowerLimit) {
        thisRestitution = (real)0.0f;
    }

    /*Combine the bounce velocity with the removed acc velocity*/
    desiredDeltaVelocity = -contactVelocity.x - thisRestitution * 
    (contactVelocity.x - velocityFromAcc);

}

void Contact::calcFrictionlessImpulse(){
    /*Build a vec that shows the change in veloocity in world 
    space for a unit imp in dir of contact norm*/
    Vector3 deltaVelWorld = relativeContactPosition[0] % contactNormal;
    Vector3 impulseContact;

    deltaVelWorld = body[0]->getInvInertiaTensorWorld().transform(deltaVelWorld);
    deltaVelWorld = deltaVelWorld % relativeContactPosition[0];

    //Work out change in velocity in contact coordinates
    real deltaVelocity = deltaVelWorld * contactNormal;

    //Add the linear comp
    deltaVelocity += body[0]->getInverseMass();

    //Check whether we need to considernthe second body's data
    if (body[1]) {
        //Go through the same transformation sequence again
    Vector3 deltaVelWorld = relativeContactPosition[1] % contactNormal;
    deltaVelWorld = body[1]->getInvInertiaTensorWorld().transform(deltaVelWorld);
    deltaVelWorld = deltaVelWorld % relativeContactPosition[1];

    //Add the change in velocity due to rotation.
    deltaVelocity += deltaVelWorld * contactNormal;

    deltaVelocity += body[1]->getInverseMass();
    }

    impulseContact.x = desiredDeltaVelocity / deltaVelocity;
    impulseContact.y = impulseContact.z = 0;

    impulse  = contactToWorld.transform(impulseContact);
}

void Contact::applyImpulse(Matrix3 &inverseInertiaTensor){
    /*Calculate change in velocity as dv = g/m (g=impulse)*/
    velocityChange[0] = impulse * body[0]->getInverseMass();

    /*Calculate the change in rotation as d(theta) = (I^(-1))u, 
    where I is the Inverse Inertia Tensor and u is the impulsive torque, 
    gotten as u= gx(q-p)*/
    Vector3 impulsiveTorque = impulse % relativeContactPosition[0];
    rotationChange[0] = body[0]->getInvInertiaTensorWorld().transform(impulsiveTorque);

    /*Apply the added angular and linear velocities*/
    body[0]->updateVelocity(velocityChange[0]);
    body[0]->updateRotation(rotationChange[0]);


    if (body[1]){
    /*Reverse direction since the impulse is in the opposite direction*/
    impulse *= -1;

    /*perform the same calculations*/
    velocityChange[1] = impulse * body[0]->getInverseMass();
    Vector3 impulsiveTorque = impulse % relativeContactPosition[1];
    rotationChange[0] = body[0]->getInvInertiaTensorWorld().transform(impulsiveTorque);

    /*Apply the added angular and linear velocities*/
    body[1]->updateVelocity(velocityChange[1]);
    body[1]->updateRotation(rotationChange[1]);
}

}

void Contact::calcRelContactPosition(){
    relativeContactPosition[0] = contactPoint - body[0]->getPosition();
    if (body[1]){
        relativeContactPosition[1] = contactPoint - body[1]->getPosition();
    }
}

void Contact::fixInterpenetration(){
/*we work out the inertia of each object in the direction of 
the contact normal, due to angular inertia only.*/
for (unsigned i=0; i<2; i++){
    if (body[i]){
        Matrix3 inverseInertiaTensor = body[i]->getInvInertiaTensorWorld();

        /*use the same as for calc frictionless velocity 
        change to work out the angular inertia*/
        Vector3 angularInertiaWorld = relativeContactPosition[i] % contactNormal;
        angularInertiaWorld = inverseInertiaTensor.transform(angularInertiaWorld);
        angularInertiaWorld = angularInertiaWorld % relativeContactPosition[i];
        angularInertia[i] = angularInertiaWorld*contactNormal;

        //The linear component is the inverse mass
        linearInertia[i] = body[i]->getInverseMass();

        /*Leep track of total inertial from all components*/
        totalInertia += linearInertia[i]+angularInertia[i];
    }
    real inverseInertia = 1 / totalInertia;

    Vector3 linearMove = contactNormal* (penetration * linearInertia[0] * inverseInertia);
    real angularMove = penetration * angularInertia[0] * inverseInertia;
    body[0]->updatePosition(linearMove);

    /*ANGULAR MOVE*/
    /*This includes three stages: 
    1. Calculate the rotation per unit movement. 
    2. Multiply by number of units needed- AngularMove, 
    3. Apply rotation to quaternion*/
    Vector3 impulsiveTorque = relativeContactPosition[0] % contactNormal;
    Vector3 impulsePerMove = body[0]->getInvInertiaTensorWorld().transform(impulsiveTorque);




    if(body[1]){
    Vector3 linearMove = contactNormal* (-penetration * linearInertia[1] * inverseInertia);
    real angularMove = -penetration * angularInertia[1] * inverseInertia;
    body[1]->updatePosition(linearMove);
    /*ANGULAR MOVE*/

    }
    }
}

void Contact::calculateInternals(real duration){
    calcRelContactPosition();
    calcContactVelocity();
    calcDesiredDeltaVelocity(duration);
}