#pragma once

/*Links connect 2 particles together, generating a contact if they are too far apart.
 The contact works opposite to the usual normal, pulling them back together instead of pushing them apart.
 It is used as a base class for cables and rods, and could be used as a base class for springs 
 with a limit to their extension*/
// #include "particle.hpp"
#include "contacts.hpp"

namespace cyclone{
class Link : public ContactGenerator {
    public:
        /*Holds the pair of bodies that are connected by this link*/
        RigidBody* body[2];

        

    protected:
        /*returns the current length of the cable*/
        real currentLength() const;

    public:
    /*Fills the given contact structure with the contact needed to keep the link from
    violatinf its constraint. The contact pointer should point to the first available 
    contact in a contact array, where limit is the maximum number of contacts in the 
    array that can be written to. The method returns the number of contacts that have 
    been written. Thus format is common to contact-generating functions, but this class
    can only generate a single contact, so the pointer can be a pointer to a single element
    . The limit parameter is assumed to be at least one (zero ain't valid), and the return
    value is either 0, if the cable wasn't overextended, or one if a contact was needed.
    */
   virtual unsigned addContact(Contact *contact, unsigned limit) const = 0;

 };

/* Cables link a pair of particles, generating a contact if they stray too far apart*/
class Cable : public Link {
    public: 
        /*Holds the max length of cable*/
        real maxLength;

        /*holds the restitution*/
        real restitution;

    public:
        /*Fille the given contact structure with the
        contact needed to keep the cable from overextending*/
        virtual unsigned addContact(Contact *contact, unsigned limit) const;
};

/*Rods link a pair of particles, with the aim of them not changing their relative positions, generating pairs of contacts on either end to ensure stability.*/
class Rod : public Link {
    public:
        /*Holds length of rod*/
        real rodLength;

    /*Fills the given contact structure with the contact needed to keep the rod from extending or compressing*/
    virtual unsigned addContact(Contact *contact, unsigned limit) const;
};

/**
* Constraints are just like links, except they connect a particle to
* an immovable anchor point.
*/
class Constraint : public ContactGenerator
{
public:
    /**
    * Holds the particles connected by this constraint.
    */
    RigidBody* body;

    /**
     * The point to which the particle is anchored.
     */
    Vector3 anchor;

protected:
    /**
    * Returns the current length of the link.
    */
    real currentLength() const {
        Vector3 length = body->getPosition() - anchor;
        return length.magnitude(); 
    }

public:
    /**
    * Geneates the contacts to keep this link from being
    * violated. This class can only ever generate a single
    * contact, so the pointer can be a pointer to a single
    * element, the limit parameter is assumed to be at least one
    * (zero isn't valid) and the return value is either 0, if the
    * cable wasn't over-extended, or one if a contact was needed.
    *
    * NB: This method is declared in the same way (as pure
    * virtual) in the parent class, but is replicated here for
    * documentation purposes.
    */
    virtual unsigned addContact(Contact *contact, unsigned limit) const = 0;
};

}