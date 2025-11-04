#pragma once
#include "body.hpp"
#include "core.hpp"
#include <set>

namespace cyclone{
/*Stores a potential contect  to check later*/
struct PotentialContact {
    /*Holds the bodies that might be in contact*/
    RigidBody* body[2];
};

/* A base class for nodes in a bounding volume heirarchy.
This class uses a binary tree to store the bounding volumes.*/
template<class BoundingVolumeClass>
class BVHNode{
    public: 
    /*Holds the child nodes of this node.*/
    BVHNode* children[2];

    /*Holds a single bounding volume encompassing all the descendants of this node*/
    BoundingVolumeClass volume;

    /*Holds the rigid body at this node of the heirarchy.
    Only leaf nodes can have a rigid body defined. (see isLeaf).
    Note that it is possible to rewrite the algorithms in this class 
    to handle objects at all levels of the heirarchy, but the code provided 
    ignores this evctor unless firstChild is NULL.*/
    RigidBody *body;

    /*Holds the parent node*/
    BVHNode *parent;

    /*Creates a new node in heirarchy with given params*/
    BVHNode(BVHNode *parent, const BoundingVolumeClass &volume, RigidBody *body=NULL):
    parent(parent), volume(volume), body(body) {
        children[0] = children[1] = NULL;
    }

    /*Checks if this node is at bottom of heirarchy.*/
    bool isLeaf() const {return (body != NULL);}

    /*Checks the potential contacts from this node downward in the heirarchy, 
    writing them to the given array (up to the given limit). Returns the number 
    of potential contacts it found.*/
    unsigned getPotentialContacts(PotentialContact *contacts, unsigned limit) const;

    bool overlaps(const BVHNode<BoundingVolumeClass> *other) const;

    unsigned getPotentialContactsWith(const BVHNode<BoundingVolumeClass> *other,
    PotentialContact *contacts,
    unsigned limit) const;

    /*Inserts the given rigid body, with the given bounding volume, 
    into the heirarchy. This may involve the creation of further bounding 
    volume nodes.*/
    void insert(RigidBody *body, const BoundingVolumeClass &volume);

    void recalculateBoundingVolume(bool recurse=true);

    /*Deletes this node, removing it first from the heirarchy, along with its associated rigid body and child nodes*/
    ~BVHNode();
};

template <class BoundingVolumeClass>
bool BVHNode<BoundingVolumeClass>::overlaps(const BVHNode<BoundingVolumeClass> *other) const 
    {return volume->overlaps(other->volume);}


template<class BoundingVolumeClass>
unsigned BVHNode<BoundingVolumeClass>::getPotentialContacts(
    PotentialContact * contacts, unsigned limit) const {
        //Early out if we don't have the room for contacts, or if we're a leaf node.
        if (isLeaf() || limit==0) return 0;

        //Get the potential contacts of one of our children with the other
        return children[0]->getPotentialContactsWith(children[1], contacts, limit);
    }


template<class BoundingVolumeClass>
unsigned BVHNode<BoundingVolumeClass>::getPotentialContactsWith(
    const BVHNode<BoundingVolumeClass> *other,
    PotentialContact *contacts,
    unsigned limit) const {
        //Early-out if we don't overlap or if we have no room to report contacts.
        if(!overlaps(other) || limit ==0) return 0;

        //If we're both at leaf nodes, then we have a potential contact.
        if (isLeaf() && other->isLeaf()) {
            contacts->body[0] = body;
            contacts->body[1] = other->body;
            return 1;
        }

        /*Determine which node to descend into. If either is a leaf, then we descend 
        the other. If both are branches, then we use the one with the largest size.*/
        if (other->isLeaf() ||
        (!isLeaf() && volume->getSize() <= other->volume->getSize())) {
            //Recurse into ourself. 
            unsigned count = children[0]->getPotentialContactsWith(other, contacts, limit);

            //Check whether we have enough slots to do the other side
            if (limit > count) {
                return count + children[1]->getPotentialContactsWith(other, contacts+count, limit-count);
            } else {
                return count;
            }
        }
        else {
            //recurse into the other node
            unsigned count = getPotentialContactsWith(other->children[0], contacts, limit);

            //Check whether we have enough slots to do the other side too.
            if (limit>count) {
                return count + getPotentialContactsWith(other->children[1], contacts+count, limit-count);
            } else {return count;}
                }
}


template<class BoundingVolumeClass>
void BVHNode<BoundingVolumeClass>::insert(
    RigidBody *newBody, const BoundingVolumeClass &newvolume){
        /*If we are a leaf, then the only option is to spawn two 
        near children and place the new body in one.*/
        if (isLeaf()){
            //Child one is a copy of this
            children[0] = new BVHNode<BoundingVolumeClass>(this, volume, body);

            //Child two holds the new body
            children[1] = new BVHNode<BoundingVolumeClass>(this, newvolume, newBody);
            
            //And now loosen the body
            this->body = NULL;

            //Recalculate bounding volume
            recalculateBoundingVolume();
        }

        /*Otherwise we work out which child gets to keep the inserted body. 
        We give it to whoever would grow the least to incorporate it.*/
        else {
            if (children[0]->volume.getGrowth(newvolume) < children[1]->volume.getGrowth(newvolume)) {
                children[0]->insert(newBody, newVolume);
            } else{
                children[1]->insert(newBody, newVolume);
            }
        }
}


template<class BoundingVolumeClass>
void BVHNode<BoundingVolumeClass>::recalculateBoundingVolume(bool recurse){
    bool recurse;
    volume = BoundingVolumeClass(children[0], children[1]);
    if (parent) parent->recalculateBoundingVolume(true);
}

template<class BoundingVolumeClass>
BVHNode<BoundingVolumeClass>::~BVHNode<BoundingVolumeClass>(){
    if (parent) {
        BVHNode<BoundingVolumeClass> *sibling;
        if (parent->children[0] == this) sibling = parent->children[1];
        else sibling = parent->children[0];

        //Writes data to our parent
        parent->volume = sibling->volume;
        parent->body = sibling->body;
        parent-children[0] = sibling->children[0];
        parent->children[1] = sibling->children[1];

        //Delete the sibling (blank parent and children to avoid processing/deleting them)
        sibling->parent = NULL;
        sibling->body=NULL;
        sibling->children[0] = NULL;
        sibling->children[1] = NULL;
        delete sibling;

        //Recalc paren'ts bounding volume
        parent->recalculateBoundingVolume();

        //Delete our children (again we remove their parent data so we don't try to process their si-[/])
        if(children[0]) {
            children[0]->parent = Null;
            delete children[0];
        }
        if (children[1]){
            children[1]->parent = NULL;
            delete children[0];
        }
    }
}



struct BoundingSphere {
    Vector3 center; 
    real radius;

    public:
    /*Creates a new bounding sphere at the given center and radius*/
    BoundingSphere(const Vector3 &center, real radius);

    /*Creates a bounding sphere to enclose the two given bounding spheres*/
    BoundingSphere(const BoundingSphere &one, const BoundingSphere &two);

    /*Checks if the bounding sphere overlaps with other given bounding sphere*/
    bool overlaps(const BoundingSphere *other) const;

    /*Returns diameter of bounding sphere*/
    real getSize() const;

    /*returns growth of bounding sphere if other is added*/
    real getGrowth(const BoundingSphere &newSphere) const;
    };


struct Plane {Vector3 position; Vector3 direction;};

struct BSPNode {
    Plane plane;
    BSPNode *front;
    BSPNode *back;

    BSPNode(){}
};

struct QuadTreeNode {
    Vector3 pos;
    QuadTreeNode *child[4];

    unsigned int getChildIndex;
};

struct OctTreeNode {
    Vector3 pos;
    OctTreeNode *child[4];
    unsigned int getChildIndex;
};

struct Grid {
    unsigned int xExtent;
    unsigned int zExtent;
    ObjectSet *locations; //An array of size xextent * yextent

    Vector3 Origin;
    Vector3 oneOverzcellSize;

    unsigned int getLocationIndex(const Vector3& object) {
    Vector3 square = object.componentProduct(oneOverzcellSize);
    return (unsigned int)(square.x) +xExtent*(unsigned int)(square.z);
    }
};

struct ObjectSet{
    std::pair<real, real> coordinates;
};
}
//TODO: Finish code from page 253
