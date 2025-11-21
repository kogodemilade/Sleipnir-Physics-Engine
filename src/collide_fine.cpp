#include "../include/Sleipnir/collide_fine.hpp"
#include <assert.h>
#include <memory>

using namespace cyclone;

void Primitive::calculateInternals(){
    transform = body->getTransform() + offset;
}

void Primitive::bindPrimitive(){
     body->setPrimitive(this);
}

bool CollisionData::hasContacts(){
    return contactsLeft > 0;
}

void CollisionData::reset(unsigned maxContacts){
    contactsLeft = maxContacts;
    contactCount = 0;
    contacts.clear();
}

void CollisionData::addContacts(unsigned count) {
    //Reduce number of contacts remaining, add number used
    contactsLeft -= count;
    contactCount += count;
}

unsigned CollisionDetector::sphereAndSphere(Sphere &one, Sphere &two, CollisionData *data){
    //Make sure we have contacts
    if(data->contactsLeft <= 0) return 0;

    one.calculateInternals();
    two.calculateInternals();

    //Cache the sphere positions
    Vector3 positionOne = one.getAxis(3);
    Vector3 positionTwo = two.getAxis(3); 


    //Find the vector between the objects
    Vector3 midline = positionOne - positionTwo;
    real size = midline.magnitude();

    /*see if large enough.*/
    if (size <=0.0f ||size >= one.radius+two.radius){
        return 0;
    }

    //We create the normal
    Vector3 normal = midline;
    normal.normalize();

    Contact contact;
    contact.contactNormal = normal;
    contact.contactPoint = positionOne + midline*(real)0.5;
    contact.penetration = (one.radius+two.radius - size);

    //Write the apprpriate data
    contact.body[0] = one.body;
    contact.body[1] = two.body;
    contact.restitution = data->restitution;
    contact.friction = data->friction;

    data->contacts.push_back(std::move(contact));
        //Point contacts to the head of the array


    data->addContacts(1);

    return 1;
}

unsigned CollisionDetector::sphereAndHalfSpace( Sphere &sphere,  Plane &plane, CollisionData *data){
    //Make sure we have contacts
    if (data->contactsLeft <= 0) return 0;

    sphere.calculateInternals();
    plane.calculateInternals();

    //cache sphere pos
    Vector3 pos = sphere.getAxis(3);

    //find distance from plane
    real ballDistance = plane.normal*pos - sphere.radius - plane.offset;



    if (ballDistance >= 0) return 0;

    //Create the contact- normal in plane's direction
    Contact contact;
    contact.contactNormal = plane.normal;
    contact.penetration = -ballDistance;
    contact.contactPoint = pos - plane.normal * (ballDistance+sphere.radius);
    contact.body[0]=sphere.body;
    contact.body[1]=NULL;
    contact.restitution = data->restitution;
    contact.friction=data->friction;

    data->contacts.push_back(std::move(contact));

    data->addContacts(1);
    return 1;
}

unsigned CollisionDetector::sphereAndTruePlane(Sphere &sphere, 
    Plane &plane, CollisionData *data ){
        //Make sure we have contacts.
        if (data->contactsLeft <= 0) return 0;

        sphere.calculateInternals();
        plane.calculateInternals();
        //Cache the sphere position.
        Vector3 pos = sphere.getAxis(3);

        //Find the distance from the plane.
        real centerDistance = plane.normal * pos - plane.offset;

        //check if we're within radius
        if (centerDistance*centerDistance > sphere.radius*sphere.radius) return 0;

        //Check which side of plane we're on
        Vector3 normal = plane.normal;
        real penetration = -centerDistance;
        if (centerDistance < 0) {
            normal *= -1;
            penetration = -penetration;
        }
        penetration += sphere.radius;

        //Create the contact - it has a normal in the plane direction

        Contact contact;
        contact.contactNormal = normal;
        contact.penetration = penetration;
        contact.contactPoint = pos - plane.normal *centerDistance;

        contact.body[0] = sphere.body;
        contact.body[1] = NULL;
        contact.restitution = data->restitution;
        contact.friction = data->friction;

        data->contacts.push_back(std::move(contact));


        data->addContacts(1);
        return 1;
    }

unsigned CollisionDetector::boxAndHalfSpace(Box &box, Plane &plane, CollisionData *data){
    box.calculateInternals();
    plane.calculateInternals();
    Vector3 halfSize = box.halfSize;
    Vector3 vertices[8] = {
        Vector3(-halfSize.x, -halfSize.y, -halfSize.z),
        Vector3(-halfSize.x, -halfSize.y, +halfSize.z),
        Vector3(-halfSize.x, +halfSize.y, -halfSize.z),
        Vector3(+halfSize.x, -halfSize.y, -halfSize.z),
        Vector3(-halfSize.x, +halfSize.y, +halfSize.z),
        Vector3(+halfSize.x, -halfSize.y, +halfSize.z),
        Vector3(+halfSize.x, +halfSize.y, -halfSize.z),
        Vector3(+halfSize.x, +halfSize.y, +halfSize.z)
    };

    unsigned contactsUsed = 0;
    Contact contact;
    for(auto vertex: vertices){
        vertex = box.offset*vertex;
        vertex = box.getTransform().transform(vertex);


        /*Calculate rhe distance from the plane.*/
        real vertexDist = vertex * plane.normal;

        /*Compare to plane's distance*/
        if (vertexDist <= plane.offset + data->tolerance) {
            //Create the conteact data
            /*The contact point is halfway between the vertex and the plane. 
            We multiply the direction by half the separation distance and 
            add the vertex location.*/
            Vector3 contactPoint = plane.normal;
            contactPoint *= 0.5 *(vertexDist - plane.offset);
            contactPoint += vertex;
            real penetration = plane.offset - vertexDist;

            contact.setData(contactPoint, plane.normal, penetration, box.body, NULL, data->restitution, data->friction);

            data->contacts.push_back(std::move(contact));
            contactsUsed++;

            if (contactsUsed >= (unsigned)data->contactsLeft) return contactsUsed;
        }
    }


    data->addContacts(contactsUsed);
    return contactsUsed;   
}


unsigned CollisionDetector::boxAndSphere(Box &box, Sphere &sphere, CollisionData *data){
    box.calculateInternals();
    sphere.calculateInternals();
    //Transform sphere center in world coordinates to box's local coordinates
    Vector3 center = sphere.getAxis(3);
    Vector3 relCenter = box.getTransform().transformInverse(center);

    if (real_abs(relCenter.x) - sphere.radius > box.halfSize.x || real_abs(relCenter.y)
         - sphere.radius > box.halfSize.y || real_abs(relCenter.z) - sphere.radius > box.halfSize.z) return 0;
    
    Vector3 closestPt(0,0,0);
    real dist;

    //Clamp each coordinate to the box.
    dist = relCenter.x;
    if (dist > box.halfSize.x) dist = box.halfSize.x;
    if (dist < -box.halfSize.x) dist = -box.halfSize.x;
    closestPt.x = dist;

    dist = relCenter.y;
    if (dist > box.halfSize.y) dist = box.halfSize.y;
    if (dist < -box.halfSize.y) dist = -box.halfSize.y;
    closestPt.y = dist;

    dist = relCenter.x;
    if (dist > box.halfSize.z) dist = box.halfSize.z;
    if (dist < -box.halfSize.z) dist = -box.halfSize.z;
    closestPt.z = dist;

    //Check we're in contact
    dist = (closestPt - relCenter).squareMagnitude();
    if (dist > sphere.radius *sphere.radius) return 0;

    //Compile the contact (transform back to world coordinates)
    Vector3 closestPtWorld = box.getTransform().transform(closestPt);
    Vector3 contactNormal = (center-closestPtWorld);
    contactNormal.normalize();
    real penetration = sphere.radius - real_sqrt(dist);

    Contact contact;
    contact.setData(closestPtWorld, contactNormal, penetration, 
        box.body, sphere.body, data->restitution, data->friction);

    data->contacts.push_back(std::move(contact));
    data->addContacts(1);
    return 1;
}


real CollisionDetector::transformToAxis(const Box &box, const Vector3 &axis){
    return box.halfSize.x * real_abs(axis*box.getAxis(0)) +
        box.halfSize.y * real_abs(axis*box.getAxis(1))+
        box.halfSize.z * real_abs(axis*box.getAxis(2));
}


bool CollisionDetector::overlapOnAxis(const Box &box1, const Box &box2, const Vector3 &axis){ //Never actually used
    /*Project the half-size of one onto axis.*/
    real oneProject = transformToAxis(box1, axis);
    real twoProject = transformToAxis(box2, axis);

    /*Find the vector between the two centers*/
    Vector3 toCenter = box2.getAxis(3) - box1.getAxis(3);

    /*Project this onto the axis*/
    real dist = real_abs(toCenter*axis);

    /*Check for overlap*/
    return (dist < oneProject+twoProject);
}


real CollisionDetector::penetrationOnAxis(const Box &one, const Box &two, const Vector3 &axis, const Vector3 &centerDist) {
    //Project the half-size of one onto axis
    real oneProject = transformToAxis(one, axis);
    real twoProject = transformToAxis(two, axis);

    //Project the centre to axis
    real distance = real_abs(centerDist * axis);

    /*Return the overlap (ie pos indicates overlap, neg indicates separation)*/
    return oneProject + twoProject - distance;
}


bool CollisionDetector::tryAxis(const Box &one, const Box &two, Vector3 axis, const Vector3 &centerDist, 
    unsigned index, real& smallestPenetration, unsigned &smallestCase){ //Use reference for real and unsigned because we're modifying them.
        //Make sure we have a normalized axis, and don't check almost parallel axes.
        if (axis.squareMagnitude() < 0.0001) return true;
        axis.normalize();

        /*Get the value of interpenetration on this axis*/
        real penetration = penetrationOnAxis(one, two, axis, centerDist);

        /*If penetration is negative, the objects are not in contact.*/
        if (penetration < 0) return false;

        /*Update the smallestPenetration and the smallest case*/
        if (penetration < smallestPenetration) {
            smallestPenetration = penetration;
            smallestCase = index;
        }
        return true;
}

//TODO: This may be useless. Review in future
unsigned CollisionDetector::checkOverlap(const Box& box1, const Box& box2, const Vector3& centerDist, Vector3 axis, unsigned index, real pen, unsigned best){
    if (!tryAxis(box1, box2, axis, centerDist, index, pen, best)) return 0;
    return 1;
}


void CollisionDetector::fillPointFaceBox(const Box &box1, const Box &box2, const Vector3 &centerDist, CollisionData *data, unsigned best, real pen){
    Contact contact;

    /*We know which axis the collision is on (i.e best),
    but we need to work out which of the two faces on this axis*/
    Vector3 normal = box1.getAxis(best);
    if (box1.getAxis(best) * centerDist > 0) {
        normal = normal * -1.0f;
    }

    /*Work out which vertex of box two we're colliding with*/
    Vector3 vertex = box2.halfSize;
    if (box2.getAxis(0) * normal < 0) vertex.x = -vertex.x;
    if (box2.getAxis(1) * normal < 0) vertex.y = -vertex.y;
    if (box2.getAxis(2) * normal < 0) vertex.z = -vertex.z;

    //Create the contact data
    contact.setData(box2.getTransform() * vertex, normal, pen, box1.body, box2.body, data->restitution, data->friction);
    data->contacts.push_back(std::move(contact));
}


Vector3 contactPoint(
    const Vector3 &pOne, const Vector3 &dOne, real oneSize, const Vector3 &pTwo, const Vector3 &dTwo, real twoSize,
    /*If true, and contact point is outside edge, then we use one's midpooint. otherwise we use two*/ bool useOne){
        Vector3 toSt, cOne, cTwo;
        real dpStaOne, dpStaTwo, dpOneTwo, smOne, smTwo;
        real denom, mua, mub;

        smOne = dOne.squareMagnitude();
        smTwo = dTwo.squareMagnitude();
        dpOneTwo = dTwo *dOne;
        toSt = pOne - pTwo;
        dpStaOne = dOne *toSt;
        dpStaTwo = dTwo * toSt;

        denom = smOne * smTwo - dpOneTwo * dpOneTwo;

        /*Zero denominator indicates parrallel lines*/
        if (real_abs(denom) < 0.0001f){
            return useOne?pOne:pTwo;
        }

        mua = (dpOneTwo * dpStaTwo - smTwo * dpStaOne)/denom;
        mub - (smOne*dpStaTwo - dpOneTwo*dpStaOne)/denom;

        /*If either of edges has nearest point out of bounds, then 
        edges aren't corssed, we have an edge-face contact. Our point 
        is on the edge, which we know from useOne param*/
        if (mua > oneSize || mua < -oneSize || mub > twoSize || mub < -twoSize){
            return useOne?pOne:pTwo;
        }
        else {
            cOne = pOne + dOne*mua;
            cTwo = pTwo + dTwo*mub;
            return cOne * 0.5 + cTwo * 0.5;
        }
    }


unsigned CollisionDetector::boxAndBox(Box &box1, Box &box2, CollisionData *data){

    box1.calculateInternals();
    box2.calculateInternals();
    //Find the vector between two centres
    Vector3 centerDist = box2.getAxis(3) - box1.getAxis(3);

    //We start assuming there is no contact
    real pen = REAL_MAX;
    unsigned best = 0xffffff;

    /*Now we check each axes, returning if it gives us a 
    separating axis, and keeping track of the axis with the 
    smallest penetration otherwise.*/
    checkOverlap(box1, box2, centerDist, box1.getAxis(0), 0, pen, best);
    checkOverlap(box1, box2, centerDist, box1.getAxis(1), 1, pen, best);
    checkOverlap(box1, box2, centerDist, box1.getAxis(2), 2, pen, best);

    checkOverlap(box1, box2, centerDist, box2.getAxis(0), 3, pen, best);
    checkOverlap(box1, box2, centerDist, box2.getAxis(1), 4, pen, best);
    checkOverlap(box1, box2, centerDist, box2.getAxis(2), 5, pen, best);

    /*Store bext axis-major, in case we run 
    into almost parallel edge collisions later*/
    unsigned bestSingleAxis = best;

    //Check overlap between 9 other parallel axes by finding cross product of each
    checkOverlap(box1, box2, centerDist, box1.getAxis(0) % box2.getAxis(0), 6, pen, best);
    checkOverlap(box1, box2, centerDist, box1.getAxis(0) % box2.getAxis(1), 7, pen, best);
    checkOverlap(box1, box2, centerDist, box1.getAxis(0) % box2.getAxis(2), 8, pen, best);

    checkOverlap(box1, box2, centerDist, box1.getAxis(1) % box2.getAxis(0), 9, pen, best);
    checkOverlap(box1, box2, centerDist, box1.getAxis(1) % box2.getAxis(1), 10, pen, best);
    checkOverlap(box1, box2, centerDist, box1.getAxis(1) % box2.getAxis(2), 11, pen, best);

    checkOverlap(box1, box2, centerDist, box1.getAxis(2) % box2.getAxis(0), 12, pen, best);
    checkOverlap(box1, box2, centerDist, box1.getAxis(2) % box2.getAxis(1), 13, pen, best);
    checkOverlap(box1, box2, centerDist, box1.getAxis(2) % box2.getAxis(2), 14, pen, best);

    /*Make sure we have a result*/
    assert(best != 0xffffff);

    /*We now know there's a collision, and we know 
    which of the axes gave the smallest penetration.
    We can now deal with it in different ways depending on the case*/
    if (best < 3) {
        //We've got a vertex of box two on a face of box one.
        fillPointFaceBox(box2, box1, centerDist*-1.0f, data, best, pen);
        data->addContacts(1);
        return 1;
    }
    else if (best < 6) {
        /*We've got a vertex of box one on a face of box 2. 
        We use the same algorithm as above, but swap around one and two 
        (and therefore also the vector between their centres)*/
        fillPointFaceBox(box1, box2, centerDist*-1.0f, data, best-3, pen);
        data->addContacts(1);
        return 1;
    }
    else {
        //We've got an edge-edge contact. Find out which axes
        best -= 6;
        unsigned oneAxisIndex = best / 3; //TODO: Check this
        unsigned twoAxisIndex = best % 3;
        Vector3 oneAxis = box1.getAxis(oneAxisIndex);
        Vector3 twoAxis = box2.getAxis(twoAxisIndex);
        Vector3 axis = oneAxis % twoAxis;
        axis.normalize();

        //Axis should point from box one to box two.
        if (axis * centerDist > 0) axis = axis * -1.0f;

        /*We have axes, but not the edges: each axis has 
        4 edges parallel to it, we need to find out which for each object.
        We do that by finding the point in the centre of the edge. We know 
        its component in the direction of the box's collision axis is zero 
        (mid point) and we det which of the extremes in each of the other axes 
        is closest*/
        Vector3 ptOnOneEdge = box1.halfSize;
        Vector3 ptOnTwoEdge = box2.halfSize;

        for (unsigned i = 0; i < 3; i++){
            if (i == oneAxisIndex) ptOnOneEdge[i] = 0;
            else if (box1.getAxis(i)*axis>0) ptOnOneEdge[i] = -ptOnOneEdge[i];

            if (i == twoAxisIndex) ptOnTwoEdge[i] = 0;
            else if (box2.getAxis(i)*axis<0) ptOnOneEdge[i] = -ptOnOneEdge[i];
        }

        //Move them into world coordinates
        ptOnOneEdge = box1.getTransform()*ptOnOneEdge;
        ptOnTwoEdge = box2.getTransform()*ptOnTwoEdge;

        /*So we have a point and a direction for the colliding edges. We need 
        to find out the point of closest approach of the two line-segments*/
        Vector3 vertex = contactPoint(ptOnOneEdge, oneAxis, box1.halfSize[oneAxisIndex], 
            ptOnTwoEdge, twoAxis, box2.halfSize[twoAxisIndex], bestSingleAxis>2);


        /*We can fill the contact*/
        Contact contact;
        contact.setData(vertex, axis, pen, box1.body, box2.body, data->restitution, data->friction);
        data->contacts.push_back(std::move(contact));

        data->addContacts(1);
        return 1;
    }
    return 0;

}

unsigned CollisionDetector::boxAndPoint(const Box &box, const Vector3 &point, CollisionData *data){
    //Transform the point into box coordinates
    Vector3 relPt = box.getTransform().transformInverse(point);

    Vector3 normal;

    //Check each axis, looking for the axis on which the penetration is least deep.
    real min_depth = box.halfSize.x - real_abs(relPt.x);
    if (min_depth < 0) return 0;
    normal = box.getAxis(0)*((relPt.x < 0) ?-1:1);

    real depth = box.halfSize.y - real_abs(relPt.y);
    if (min_depth < 0) return 0;
    else if (depth<min_depth){
        min_depth = depth;
        normal=box.getAxis(1)*((relPt.y<0)?-1:1);
    }

    depth = box.halfSize.z - real_abs(relPt.z);
    if (min_depth < 0) return 0;
        else if (depth<min_depth){
        min_depth = depth;
        normal=box.getAxis(2)*((relPt.z<0)?-1:1);
    }

    //Compile the contact
    Contact contact;
    contact.setData(point, normal, min_depth, box.body, NULL, data->restitution, data->friction);
    data->contacts.push_back(std::move(contact));
    data->addContacts(1);
    /*Note that we don't know what rigid body the point belongs to, so we just use NULL*/
    return 1;
}