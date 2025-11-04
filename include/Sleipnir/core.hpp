#pragma once

#include "precision.hpp"
#include <initializer_list>
#include <algorithm>
#include <iterator>

namespace cyclone{
// Holds a vector in 3 dimensions. Four data members are allocated to ensure alignment in an array.
class Matrix3; 
class Matrix4;
class Quaternion;

class Vector3 {
    public:
        real x;
        real y;
        real z;

    private: 
    // Padding to ensure 4-word alignment.
        real pad;

    public:
    // The default constructor 
        Vector3(): x(0), y(0), z(0) {}

        /*The explicit constructor creates a vector with the given components.*/
        Vector3(const real x, real y, real z) : x(x), y(y), z(z) {}


    // Flips all the components of the vector. 
    void invert(){
        x = -x;
        y = -y;
        z = -z;
    }

    /* Gets the magnitude of this vector. */
    real magnitude() const{
        return real_sqrt(x*x+ y*y + z*z);
    }

    /* Gets the square magnitude of this vector. */
    real squareMagnitude() const {
        return x*x+y*y+z*z;
    }


    /*Multiplies this vector by the given scalar.*/
    void operator *=(const real value){
        x*=value;
        y*=value;
        z*=value;
    }

    /*Returns a copy of this vector scaled to the given value*/
    Vector3 operator*(const real value) const{
        return Vector3(x*value, y*value, z*value);
    }

    real operator[](int i) const{
        if (i==0) return (*this).x;  
        else if (i==1) return y;
        return z;
    }

    real& operator[](int i){
        if (i==0) return (*this).x;  
        else if (i==1) return y;
        return z;
    }


    /*Turns a non-zero vector into a vector into a vector of unit length*/
    void normalize(){
        real l = magnitude();
        if (l>0){
            (*this)*=((real)1)/l;
        }
    }


    /*Adds the given vector to this.*/
    void operator +=(const Vector3 &v){
        x += v.x;
        y += v.y;
        z += v.z;
    }

    /*Returns a copy/value of the given vector added to this*/
    Vector3 operator +(const Vector3 &v)const{
        return Vector3(x+v.x, y+v.y, z+v.z);
    }


    /*Subtracts the given vector FROM this one*/
    void operator -=(const Vector3 &v){
        x -= v.x;
        y -= v.y;
        z -= v.z;
    }

    /*Returns a copy/value of the given vector subtracted from this*/
    Vector3 operator -(const Vector3 &v)const{
        return Vector3(x-v.x, y-v.y, z-v.z);
    }

    /*Adds the given vector to this, scaled by the given amount.*/
    void addScaledVector(const Vector3& vector, real scale){
        x += scale*vector.x;
        y += scale*vector.y;
        z += scale*vector.z;
    }



    /**Calculates and returns a component-wise product of this vector and a given vector */
    Vector3 componentProduct(const Vector3 &vector) const{
        return Vector3(x* vector.x, y*vector.y, z*vector.z);
    }

    /**Calculates a component-wise product of this vector and a given vector and sets this vector to the result */
    void componentProductUpdate(const Vector3 &vector){
        x *= vector.x;
        y *= vector.y;
        z *= vector.z;
    }

    /**Calculates the dot product of this vector and a given vector and returns the result */ 
    real scalarProduct(const Vector3 &vector) const {
        return real(x*vector.x + y*vector.y + z*vector.z);
    }

    /**Calculates the dot product of this vector and a given vector and returns the result */ 
    real operator *(const Vector3 &vector)const {
        return real(x*vector.x + y*vector.y + z*vector.z);
    }

    /**Calculates the cross product of this vector and a given vector and returns the result */
    Vector3 vectorProduct(const Vector3 &vector) const{
        return Vector3(y*vector.z - z*vector.y, 
                       x*vector.z - z*vector.x,
                       x*vector.y - y*vector.x);
    }

    /**Calculates the cross product of this vector and a given vector and returns the result */
    Vector3 operator %(const Vector3 &vector) const{
        return Vector3(y*vector.z - z*vector.y, 
                       x*vector.z - z*vector.x,
                       x*vector.y - y*vector.x);
    }

    /**Updates this vector to the vector product of itself and the given vector */
    void operator %= (const Vector3 &vector){
        *this = vectorProduct(vector);
    }

    //Returns a transformed a Vector from local positioning to global positioning
    Vector3 localToWorld(const Vector3 &local, const Matrix4 &transform) const;

    //Transforms given vector from local positioning to global positioning
    void setWorldVec(const Matrix4 &transform);

    //Returns a transformed a Vector from global positioning to local positioning
    Vector3 worldToLocal(const Vector3 &world, const Matrix4 &transform) const;

    //Transforms given vector from global positioning to local positioning
    void setLocalVec(const Matrix4 &transform);

    //Returns a transformed a Vector from local directions to global directions
    Vector3 localToWorldDirn(const Vector3 &local, const Matrix4 &transform) const;

    //Returns a transformed a Vector from global directions to local directions
    Vector3 worldToLocalDirn(const Vector3 &world, const Matrix4 &transform) const;

    void setLocalVecDirn(const Matrix4 &transform);

    void setWorldVecDirn(const Matrix4 &transform);

    //zero all components of vector
    void clear(){
        x=y=z=0;
    }

};

/**Holds an inertia tensor, consisting of a 3x3 row-major matrix. This matrix is not padding to
 * produce an aligned structure, since it is most commonly used with a mass and 2 damping coeffs to 
 * make the 12-element characteristics array of a rigid body.
 */

 class Matrix3 {
    public:
    /*Holds the tensor matrix data in array form*/
    real data[9];

    //Constructor without params
    Matrix3();

    //Constructor with a list of reals
    // Matrix3(std::initializer_list<real> init) {
    //     std::copy(init.begin(), init.begin()+std::min(size_t)9, init.size(), data);
    // }

    /*Sets the columns of the matrix, where a, b and c are column vectors*/
    void setComponents(const Vector3 &a, const Vector3 &b, const Vector3 &c);
    /*Returns the result of a 3x3 Matrix multiplied by the given column vector*/
    Vector3 operator *(const Vector3 &vec) const;

    /*Alias for the * operator
    @see operator **/
    Vector3 transform(const Vector3 &vec) const;

    /*Returns the result of the multiplication of this matrix and the given matrix.*/
    Matrix3 operator *(Matrix3 other) const;

    /*Sets the inverse of the given 3x3 matrix*/
    void setInverse(const Matrix3 &m);

    //Returns a new matrix containing the inverse of this matrix
    Matrix3 inverse() const;

    //Inverts this matrix
    void invert() {
        setInverse(*this);
    }

    /*Sets matrix to be transpose of given matrix*/
    void setTranspose(const Matrix3 &m);

    /*Returns a new matrix containing the transpose of this matrix*/
    Matrix3 transpose() const;

    /*transforms a vector by the transpose of this matrix*/
    Vector3 transformTranspose(const Vector3 &vec) const;

    /*Sets this matrix to be the rotation matrix corresponding to the given quaternion*/
    void setOrientation(const Quaternion &q);
 };


/*Holds an inertia tensor, consisting of a rotation matrix and a position. The matrix
has 12 elements; it is assumed that the remaining four are (0, 0, 0 ,1), producing a
homogeneuous matrix.*/
class Matrix4 {
    public:
    /*Holds the transform matrix data in array form*/
    real data[12];

    Matrix4();
    /*@param vector The vector to transform*/
    Vector3 operator*(const Vector3 &vec) const;

    //Returns a matrix that is this matrix multiplied by the given other matrix
    Matrix4 operator*(const Matrix4 &o) const;

    /*returns deeterminant of matrix*/
    real getDeterminant() const;

    /*Gets a vector representing one axis (i.e one column) in the matrix.
    @param i the row to return. Row 3 corresponds to the position of the transform matrix.
    @return The vector*/
    Vector3 getAxisVector(int i) const; 

    /**Sets the matrix to be the inverse of given matrix */
    void setInverse(const Matrix4 &m);

    /*Inverts matrix*/
    void Invert() {
        setInverse(*this);
    }

    //Sets mat to be rot mat corresponding to given quaternion
    void setOrientAndPos(const Quaternion &q, const Vector3 &pos);

    /*transform the given vector by the transformational inverse of this matrix*/
    Vector3 transformInverse(const Vector3 &vector) const;

    /*Transform given vector by matrix*/
    Vector3 transform(const Vector3 &vector) const;


    //Transform the given direction vector by this matrix.
    Vector3 transformDirection(const Vector3 &vector) const;

    //Transform the given direction vector by the transformational inverse of this matrix;
    Vector3 transformInvDir(const Vector3 &vector) const;

};

/*Holds a quaternion, a 4D number used to represent rotations and orientations. A quaternion has 4 elements, w, x,y and z*/
class Quaternion {
    public:
        union 
        {
            struct {
        //Holds the w element
        real w;

        //Holds the x element 
        real x;

        //Holds the y element
        real y;

        //Holds the z element
        real z;
            };
        //Holds data in array form
        real data[4];
        };
        


    /*Creates a new instance of the quaternion class with 4 real numbers as arguments*/
    Quaternion(real w, real x, real y, real z);

    /*Creates a new instance of the quaternion class with a C-style array of length 4, 
    with the members in the order: w, x, y, z*/
    Quaternion(real data[4]);

    /*Creates a new instance of the quaternion class with 4 real numbers */
    Quaternion();

    /*Normalizes the quaternion to unit length, making it a valid orientation quaternion*/
    void normalize();

    /*Multiplies the quaternion by the given quaternion.*/
    void operator *=(const Quaternion &multiplier);

    /*Rotate a quaternion by a vector*/
    void rotateByVector(const Vector3 &vector);

    /*Adds the given vector to this, scaled by the given amount.
    This is used to update the orientation quaternion by a rotation and time.
    @param vector The vector to add.
    @param scale The amount of the vector to add.*/
    void addScaledVector(const Vector3& vector, real scale);
};
}


/*TODO: 
*Make a matrix transformation function that handles transformations between different bases: Mt prime = MbMtMb^(-1)
*Normalize the commenting style, using multi-line comments for all comments 
*Move all function definition (minus declaration) to core.cpp */