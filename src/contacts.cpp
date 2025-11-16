#include "../include/Sleipnir/contacts.hpp"
#include <memory>
#include <vector>

using namespace cyclone;

void ContactResolver::resolveContacts(std::vector<Contact> &contacts, unsigned numContacts, real duration, unsigned numPosIterations, unsigned numVelocityIterations){
    /*Make sure we have something to do.*/
    if (numContacts == 0) return;

    /*Prepare the contacts for processing*/
    prepareContacts(contacts, numContacts, duration);

    /*Resolve interpenetrations first*/
    adjustPositions(contacts, numContacts, duration, numPosIterations);

    /*Resolve the velocity */
    adjustVelocities(contacts, numContacts, duration, numVelocityIterations);
}

void ContactResolver::prepareContacts(std::vector<Contact> &contacts, unsigned numContacts, real duration){
    /*Generate contact velocity, relative positions for torque 
    and contactToWorld .*/
    for (auto &contact : contacts){
        /*Calculate the internal contact data (inertia, basis, etc)*/
        contact.calculateInternals(duration);
    }
}

void ContactResolver::adjustPositions(std::vector<Contact> &contacts, unsigned numContacts, real duration, unsigned numPosIterations){
    unsigned positionIterations = numPosIterations;
    for (unsigned j=0; j < positionIterations; j++){
        Contact* worstContact = nullptr;
        real worstPenetration = penetrationEpsilon;
        for (auto& contact: contacts){
            if(contact.penetration > worstPenetration){
                worstContact = &contact;
                worstPenetration = contact.penetration;
            }
        }
        // for(Contact* contact=contacts; contact<lastContact; contact++){
        //     if (contact->penetration > worstPenetration){
        //         worstContact = contact;
        //         worstPenetration = contact->penetration;
        //     }
        // }

        if (worstContact) {
        worstContact->matchAwakeState();
        worstContact->fixInterpenetration();
        //this action may have changed the penetration of other bodies, so we update contacts.
        // unsigned i=0
        for(unsigned i=0; i<numContacts;i++){
            Vector3 cp; //CENTER POINT
            // if(&contacts[i] == worstContact) continue;
            if(contacts[i].body[0]){ //fix code here. we've moved from raw pointers to vectors.
                if(contacts[i].body[0]==worstContact->body[0]){
                    /*Linear change due to angular velocity*/
                    cp = worstContact->angularChange[0].vectorProduct(contacts[i].relativeContactPosition[0]);
                    /*Linear change due to linear velocity*/
                    cp+=worstContact->linearChange[0];

                    /*Shift it alonng the contact normal*/
                    contacts[i].penetration -= cp.scalarProduct(contacts[i].contactNormal); //CHECK THIS

                } else if (contacts[i].body[0] == worstContact->body[1]){
                    cp = worstContact->angularChange[1].vectorProduct(contacts[i].relativeContactPosition[0]);
                    cp += worstContact->linearChange[1];
                    contacts[i].penetration -= cp.scalarProduct(contacts[i].contactNormal);
                }
            }
            if(contacts[i].body[1]){
                if(contacts[i].body[1]==worstContact->body[0]){
                    cp = worstContact->angularChange[0].vectorProduct(contacts[i].relativeContactPosition[1]);
                    cp+=worstContact->linearChange[0];
                    contacts[i].penetration += cp.scalarProduct(contacts[i].contactNormal); //CHECK THIS

                } else if (contacts[i].body[1] == worstContact->body[1]){
                    cp = worstContact->angularChange[1].vectorProduct(contacts[i].relativeContactPosition[1]);
                    cp += worstContact->linearChange[1];
                    contacts[i].penetration += cp.scalarProduct(contacts[i].contactNormal);
                    }
            }
        else continue;
        }   
    } else break; //new addition
}
}

void ContactResolver::adjustVelocities(std::vector<Contact> &contacts, unsigned numContacts, real duration, unsigned numVelIterations){
    unsigned VelocityIterations = numVelIterations;
    for (unsigned j = 0; j < numVelIterations; j++){
        Contact *worstContact = nullptr;
        real fastestClosingVelocity = velocityEpsilon;

        for (auto& contact: contacts){
            if(contact.desiredDeltaVelocity > fastestClosingVelocity){
                worstContact = &contact;
                fastestClosingVelocity = contact.desiredDeltaVelocity;
            }
        }

        if (!worstContact) break;
        worstContact->matchAwakeState();
        worstContact->applyImpulse();

        /*This may have changed the closing velocities of other bodies*/
        for(unsigned i=0; i<numContacts;i++){
            Vector3 cp;
            if(contacts[i].body[0]){ //fix code here. we've moved from raw pointers to vectors.
                if(contacts[i].body[0]==worstContact->body[0]){
                    /*Linear change due to angular velocity*/
                    cp = worstContact->rotationChange[0].vectorProduct(contacts[i].relativeContactPosition[0]);
                    /*Linear change due to linear velocity*/
                    cp+=worstContact->velocityChange[0];

                    /*Shift it alonng the contact normal*/
                    contacts[i].contactVelocity += contacts[i].contactToWorld.transformTranspose(cp); //CHECK THIS
                    contacts[i].calcDesiredDeltaVelocity(duration);

                } else if (contacts[i].body[0] == worstContact->body[1]){
                    cp = worstContact->rotationChange[0].vectorProduct(contacts[i].relativeContactPosition[0]);
                    cp+=worstContact->velocityChange[0];
                    contacts[i].contactVelocity -= contacts[i].contactToWorld.transformTranspose(cp); //CHECK THIS
                    contacts[i].calcDesiredDeltaVelocity(duration);
                }
            }
            if(contacts[i].body[1]){
                if(contacts[i].body[1]==worstContact->body[0]){
                    cp = worstContact->rotationChange[0].vectorProduct(contacts[i].relativeContactPosition[1]);
                    cp+=worstContact->velocityChange[0];
                    contacts[i].contactVelocity -= contacts[i].contactToWorld.transformTranspose(cp); //CHECK THIS
                    contacts[i].calcDesiredDeltaVelocity(duration);

                } else if (contacts[i].body[1] == worstContact->body[1]){
                    cp = worstContact->rotationChange[1].vectorProduct(contacts[i].relativeContactPosition[1]);
                    cp+=worstContact->velocityChange[1];
                    contacts[i].contactVelocity += contacts[i].contactToWorld.transformTranspose(cp); //CHECK THIS
                    contacts[i].calcDesiredDeltaVelocity(duration);
                    }
            }
        else continue;
        }  
    }
}

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

void Contact::makeOrthonormalBasis(const Vector3 &x, Vector3 &y, Vector3 &z){
    //Default y to (0,1,0)
    y = Vector3(0, 1, 0);
    z = x%y;
    z.normalize();
    if (z.magnitude() < 0.001){
        y = Vector3(1, 0, 0);
        z = x%y; 
        z.normalize();
    }
    y = x%z;
    y.normalize();
    /*Scaling factor to ensure the results are normalized.*/
    // const real s = 1.0/real_sqrt(x.z*x.z + x.x*x.x);

    // //The new Z axis is at right agles to the world Y axis.
    // z.x = x.z*s;
    // z.y = 0;
    // z.z = -x.x*s;


    // //The new Y axis is at right angles to the new X and Z axes
    // y.x = x.y*z.x;
    // y.y = x.z*z.x - x.x*z.z;
    // y.z = -x.y*z.x;
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

void Contact::matchAwakeState(){
    /*Collisions with the word never cause a body to wake up*/
    if (!body[1]) return;

    bool body0awake = body[0]->getState();
    bool body1awake = body[1]->getState();

    //Wake up only the sleeping one
    if (body0awake ^ body1awake){
        if (body0awake) body[1]->setAwake(1);
        else body[0]->setAwake(1);
    }
}

void Contact::calcContactVelocity(real duration){
    Vector3 deltaVelocityVec = body[0]->getRotation() % relativeContactPosition[0];
    deltaVelocityVec += body[0]->getVelocity();

    /*Calculate the amount of velocity that is due to forces without reactions*/
    Vector3 accVelocity = body[0]->getPrevAcceleration()*duration;

    if(body[1]) {
        deltaVelocityVec -= body[1]->getRotation() % relativeContactPosition[1];
        deltaVelocityVec -= body[1]->getVelocity();

    /*Calculate the amount of velocity that id due to forces without reactions*/
    accVelocity -= body[1]->getPrevAcceleration()*duration;
    }
    contactVelocity = contactToWorld.transformTranspose(deltaVelocityVec);


    
    /*Calculate the velocity in contact coordinates*/
    accVelocity = contactToWorld.transformTranspose(accVelocity);
    
    /*Ignore any component of acceleration in the contact 
    normal direction. Only planar acceleration*/
    accVelocity.x = 0;


    /*Add the planar vel - if there's enough friction 
    they will be removed during velocity resolution*/
    contactVelocity += accVelocity;
}

void Contact::calcDesiredDeltaVelocity(real duration){
    const static real velLowerLimit = (real)0.25f;

    //Get velocity from acceleration, V = u + at
    real velocityFromAcc = 0;
    velocityFromAcc += body[0]->getPrevAcceleration() * duration *contactNormal;
    
    if (body[1]) {
        velocityFromAcc -= body[1]->getPrevAcceleration() * duration * contactNormal;
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

void Contact::calcImpulse(){
    /*Build a matrix that shows the change in veloocity in world 
    space for a unit imp in dir of contact norm. 
    The equivalent of cross product between vectors is multiplication by a skew matrix*/
    Matrix3 impulseToTorque = relativeContactPosition[0].skewSymmetricMatrix();

    Matrix3 deltaVelWorld = impulseToTorque;
    deltaVelWorld *= body[0]->getInvInertiaTensorWorld();
    deltaVelWorld *= impulseToTorque.transpose();
    deltaVelWorld *= -1;

    /*linear component. The matrix form of a real is a diagonal matrix 
    of all values = k (others = 0)*/
    real inverseMass = body[0]->getInverseMass();
    deltaVelWorld.data[0] += inverseMass;
    deltaVelWorld.data[4] += inverseMass;
    deltaVelWorld.data[8] += inverseMass;
    /*Check if body 2's data is available*/
;    if (body[1]){
        //Find inertia tensor
        /*Set cross product matrix*/
        Matrix3 impToTorque = relativeContactPosition[1].skewSymmetricMatrix();
        
        /*Find inertai tensor for this body*/
        Matrix3 invInertiaTensor = body[1]->getInvInertiaTensorWorld();
        Matrix3 deltaVelWorld2 = impToTorque * invInertiaTensor;
        deltaVelWorld2 *= impToTorque.transpose();
        deltaVelWorld2 *=-1;

        real inverseMass = body[1]->getInverseMass();
        deltaVelWorld2.data[0] += inverseMass;
        deltaVelWorld2.data[4] += inverseMass;
        deltaVelWorld2.data[8] += inverseMass;

        deltaVelWorld = deltaVelWorld+ deltaVelWorld2;
    }

    Vector3 impulseContact;
    /*change to contact coord by change of basis */

    Matrix3 deltaVelocity = contactToWorld.transpose() * deltaVelWorld;
    deltaVelocity *= contactToWorld;

    /*Invert matrix to get impulse per unit velocity*/
    Matrix3 impulseMatrix = deltaVelocity.inverse();

    /*Find target velocities to kill*/
    Vector3 velKill(desiredDeltaVelocity-contactVelocity.x, -contactVelocity.y, -contactVelocity.z);

    //Find the impulse to kill target velocities
    impulseContact = impulseMatrix.transform(velKill);

    /*Check for exceeding friction on contact plane (NOT normal).*/
    real planarImpulse = real_sqrt(impulseContact.y*impulseContact.y + impulseContact.z*impulseContact.z);

    /*if Fplanar <= coeffOfFriction * normal force, use static friction. Else, use dynamic friction*/
    if (planarImpulse > impulseContact.x*friction) {
        /*We need to use dynamic friction*/
        //Normalize the y and z impulses to get their direction

        impulseContact.y /= planarImpulse;
        impulseContact.z /= planarImpulse;

        impulseContact.x = deltaVelocity.data[0] + deltaVelocity.data[1]*friction*impulseContact.y + deltaVelocity.data[2]*friction*impulseContact.z; //Check this when debugging. May be wrong
        impulseContact.x = desiredDeltaVelocity / impulseContact.x;
        impulseContact.y *= friction * impulseContact.x;
        impulseContact.z *= friction * impulseContact.x;
    }

    impulse  = contactToWorld.transform(impulseContact);
}

void Contact::applyImpulse(){
    calcImpulse();
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
    velocityChange[1] = impulse * body[1]->getInverseMass();
    Vector3 impulsiveTorque = impulse % relativeContactPosition[1];
    rotationChange[1] = body[1]->getInvInertiaTensorWorld().transform(impulsiveTorque);

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

void Contact::fixInterpenetration(){ //TODO: Code for this function is pretty ugly. Clean up
/*we work out the inertia of each object in the direction of 
the contact normal, due to angular inertia only.*/

/*Limit for amount of angular Movement caused by resolving interpenetration*/
real angularLimitConstant = (real)0.2f;
real limit = angularLimitConstant * relativeContactPosition[0].magnitude();
totalInertia = 0;

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
    } }
    real inverseInertia = 1 / totalInertia;

    /*LINEAR MOVE*/
    /*We find the linear motion based on the equation relating impulse to velocity*/ //TODO
    real linearMove = (penetration * linearInertia[0] * inverseInertia);
    

    /*ANGULAR MOVE*/
    /*This includes three stages: 
    1. Calculate the rotation per unit movement. 
    2. Multiply by number of units needed- AngularMove, 
    3. Apply rotation to quaternion*/
    real angularMove = penetration * angularInertia[0] * inverseInertia;

    /*Check if angular move is within limits*/
    if (real_abs(angularMove) > limit){
        real totalMove = linearMove + angularMove;
        if (angularMove >=0){
            angularMove = limit;
        } else {
            angularMove = -limit;
        }
        /*Make the linear move do the extra work*/
        linearMove = totalMove - angularMove;
    }
    Vector3 impulsiveTorque = relativeContactPosition[0] % contactNormal;
    Vector3 impulsePerMove = body[0]->getInvInertiaTensorWorld().transform(impulsiveTorque);

    /*Multiply by inertia to get one unit of movement*/
    Vector3 rotationPerMove = impulsePerMove * (1/(angularInertia[0]+0.001f));

    /*Multiply by angular move to get the total rotation*/
    Vector3 rotation = rotationPerMove*angularMove;


    Vector3 linearMoveVec = contactNormal * linearMove;
    linearChange[0] = linearMoveVec;
    rotationChange[0] = rotation;
    body[0]->updatePosition(linearMoveVec);
    body[0]->rotateByVector(rotation);



    if(body[1]){
    real limit_ = angularLimitConstant * relativeContactPosition[1].magnitude();

    real linearMove = -penetration * linearInertia[1] * inverseInertia;
    Vector3 impulsiveTorque = relativeContactPosition[1] % contactNormal;
    Vector3 impulsePerMove = body[1]->getInvInertiaTensorWorld().transform(impulsiveTorque);

    /*ANGULAR MOVE*/
    real angularMove = -penetration * angularInertia[1] * inverseInertia;

        /*Check if angular move is within limits*/
    if (real_abs(angularMove) > limit_){
        real totalMove_ = linearMove + angularMove;
        if (angularMove >=0){
            angularMove = limit_;
        } else {
            angularMove = -limit_;
        }
        /*Make the linear move do the extra work*/
        linearMove = totalMove_ - angularMove;
    }

    
    // Vector3 impulsiveTorque = relativeContactPosition[1] % contactNormal;

    /*Multiply by inertia to get one unit of movement*/
    Vector3 rotationPerMove = impulsePerMove * (1/(angularInertia[1]+0.001f));

    /*Multiply by angular move to get the total rotation*/
    Vector3 rotation = rotationPerMove*angularMove;

    Vector3 linearMoveVec = contactNormal*linearMove;
    linearChange[1] = linearMoveVec;
    rotationChange[1] = rotation;
    body[1]->updatePosition(linearMoveVec);
    body[1]->rotateByVector(rotation);
        }

    // penetration = (contactPoint - body[0]->getPosition()).magnitude();
}

void Contact::calcContactToWorld(){
    Vector3 x = contactNormal;
    Vector3 y;
    Vector3 z;
    makeOrthonormalBasis(x, y, z);
    contactToWorld.setComponents(x, y, z);
}

void Contact::calculateInternals(real duration){
    calcRelContactPosition();
    calcContactToWorld();
    calcContactVelocity(duration);
    calcDesiredDeltaVelocity(duration);
}