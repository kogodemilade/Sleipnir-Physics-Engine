#include "../include/Sleipnir/core.hpp" //CHANGE TO #include "cyclone/core.hpp"

using namespace cyclone;

Vector3 Vector3::localToWorld(const Vector3 &local, const Matrix4 &transform) const{
    return transform.transform(local);
}

Vector3 Vector3::worldToLocal(const Vector3 &world, const Matrix4 &transform) const{
    return transform.transformInverse(world);
}

void Vector3::setLocalVec(const Matrix4 &transform) {
    Vector3 tmp = worldToLocal(*this, transform);
    x = tmp.x;
    y = tmp.y;
    z = tmp.z;
}

void Vector3::setWorldVec(const Matrix4 &transform) {
    Vector3 tmp = localToWorld(*this, transform);
    x = tmp.x;
    y = tmp.y;
    z = tmp.z;
}

Vector3 Vector3::localToWorldDirn(const Vector3 &local, const Matrix4 &transform) const {
    return transform.transformDirection(local);
}

Vector3 Vector3::worldToLocalDirn(const Vector3 &world, const Matrix4 &transform) const {
    return transform.transformInvDir(world);
}

void Vector3::setLocalVecDirn(const Matrix4 &transform) {
    Vector3 tmp = worldToLocalDirn(*this, transform);
    x = tmp.x;
    y = tmp.y;
    z = tmp.z;
}

void Vector3::setWorldVecDirn(const Matrix4 &transform) {
    Vector3 tmp = localToWorldDirn(*this, transform);
    x = tmp.x;
    y = tmp.y;
    z = tmp.z;
}

Matrix3 Vector3::skewSymmetricMatrix(){
    Matrix3 res;
    res.data[0] = res.data[4] = res.data[8] = 0;
    res.data[1] = -z;
    res.data[2] = y;
    res.data[3] = z;
    res.data[5] = -x;
    res.data[6] = -y;
    res.data[7] = x;
}



Matrix4::Matrix4(){
    real data[12];
}

Vector3 Matrix4::transformDirection(const Vector3 &vector) const {
    return Vector3(
            vector.x*data[0] +vector.y * data[1] + vector.z *data[2],
            vector.x*data[4] +vector.y * data[5] + vector.z *data[6],
            vector.x*data[8] +vector.y * data[9] + vector.z *data[10]
        );
    }

Vector3 Matrix4::transformInvDir(const Vector3 &vector) const {
    return Vector3(
    vector.x * data[0] + vector.y * data[4] + vector.z * data[8],
    vector.x * data[1] + vector.y * data[5] + vector.z * data[9],
    vector.x * data[2] + vector.y * data[6] + vector.z * data[10]
    );
}

real Matrix4::getDeterminant() const {
    return data[8]*data[5]*data[2] + data[4]*data[9]*data[2]+
            data[8]*data[1]*data[6]-data[0]*data[9]*data[6]-
            data[4]*data[1]*data[10]+data[0]*data[5]*data[10];
}

Vector3 Matrix4::getAxisVector(int i) const {
    return Vector3(data[i], data[i+4], data[i+8]);
}

void Matrix4::setInverse(const Matrix4 &m) {
    //Make sure det is non-zero
    real det = getDeterminant();
    if (det == 0) return;
    det = ((real)1.0)/det;

    data[0] = (-m.data[9]*m.data[6]+m.data[5]*m.data[10])*det;

    data[4] = (m.data[8]*m.data[6]-m.data[4]*m.data[10])*det;

    data[8] = (-m.data[8]*m.data[5]+m.data[4]*m.data[9]* m.data[15])*det;

    data[1] = (m.data[9]*m.data[2]-m.data[1]*m.data[10])*det;

    data[5] = (-m.data[8]*m.data[2]+m.data[0]*m.data[10])*det;

    data[9] = (m.data[8]*m.data[1]-m.data[0]*m.data[9]* m.data[15])*det;

    data[2] = (-m.data[5]*m.data[2]+m.data[1]*m.data[6]* m.data[15])*det;

    data[6] = (+m.data[4]*m.data[2]-m.data[0]*m.data[6]* m.data[15])*det;

    data[10] = (-m.data[4]*m.data[1]+m.data[0]*m.data[5]* m.data[15])*det;

    data[3] = (m.data[9]*m.data[6]*m.data[3]
                -m.data[5]*m.data[10]*m.data[3]
                -m.data[9]*m.data[2]*m.data[7]
                +m.data[1]*m.data[10]*m.data[7]
                +m.data[5]*m.data[2]*m.data[11]
                -m.data[1]*m.data[6]*m.data[11])*det;

    data[7] = (-m.data[8]*m.data[6]*m.data[3]
                +m.data[4]*m.data[10]*m.data[3]
                +m.data[8]*m.data[2]*m.data[7]
                -m.data[0]*m.data[10]*m.data[7]
                -m.data[4]*m.data[2]*m.data[11]
                +m.data[0]*m.data[6]*m.data[11])*det;

    data[11] =(m.data[8]*m.data[5]*m.data[3]
                -m.data[4]*m.data[9]*m.data[3]
                -m.data[8]*m.data[1]*m.data[7]
                +m.data[0]*m.data[9]*m.data[7]
                +m.data[4]*m.data[1]*m.data[11]
                -m.data[0]*m.data[5]*m.data[11])*det;

}

Matrix4 Matrix4::operator*(const Matrix4 &o) const {
    Matrix4 result;
    result.data[0] = (o.data[0]*data[0]) + (o.data[4]*data[1]) +
    (o.data[8]*data[2]);
    result.data[4] = (o.data[0]*data[4]) + (o.data[4]*data[5]) +
    (o.data[8]*data[6]);
    result.data[8] = (o.data[0]*data[8]) + (o.data[4]*data[9]) +
    (o.data[8]*data[10]);
    result.data[1] = (o.data[1]*data[0]) + (o.data[5]*data[1]) +
    (o.data[9]*data[2]);
    result.data[5] = (o.data[1]*data[4]) + (o.data[5]*data[5]) +
    (o.data[9]*data[6]);
    result.data[9] = (o.data[1]*data[8]) + (o.data[5]*data[9]) +
    (o.data[9]*data[10]);
    result.data[2] = (o.data[2]*data[0]) + (o.data[6]*data[1]) +
    (o.data[10]*data[2]);
    result.data[6] = (o.data[2]*data[4]) + (o.data[6]*data[5]) +
    (o.data[10]*data[6]);
    result.data[10] = (o.data[2]*data[8]) + (o.data[6]*data[9]) +
    (o.data[10]*data[10]);
    result.data[3] = (o.data[3]*data[0]) + (o.data[7]*data[1]) +
    (o.data[11]*data[2]) + data[3];
    result.data[7] = (o.data[3]*data[4]) + (o.data[7]*data[5]) +
    (o.data[11]*data[6]) + data[7];
    result.data[11] = (o.data[3]*data[8]) + (o.data[7]*data[9]) +
    (o.data[11]*data[10]) + data[11];
    return result;
}

Vector3 Matrix4::operator*(const Vector3 &vec) const {
    return Vector3(
        (vec.x*data[0]+vec.y*data[1]+vec.z*data[2]) + data[3],
        (vec.x*data[4]+vec.y*data[5]+vec.z*data[6]) + data[7],
        (vec.x*data[8]+vec.y*data[9]+vec.z*data[10]) + data[11]
    );
}

void Matrix4::setOrientAndPos(const Quaternion &q, const Vector3 &pos){
    data[0] = 1 - (2*q.y*q.y + 2*q.z*q.z);
    data[1] = 2*(q.x*q.y + q.z*q.w);
    data[2] = 2*(q.x*q.z - q.y*q.w);
    data[3] = pos.x;

    data[4] = 2*(q.x*q.y - q.z*q.w);
    data[5] = 1 - (2*(q.z*q.x + q.z*q.z));
    data[6] = 2*(q.y*q.z + q.x*q.w);
    data[7] = pos.y;

    data[8] = 2*(q.x*q.z + q.y*q.w);
    data[9] = 2*(q.y*q.z - q.x*q.w);
    data[10] = 1 - (2*(q.x*q.x + q.y*q.y));
    data[11] = pos.z;
}

Vector3 Matrix4::transformInverse(const Vector3 &vector) const{
    Vector3 tmp = vector;
    tmp.x -= data[3];
    tmp.y -= data[7];
    tmp.z -= data[11];

    return Vector3(
        tmp.x * data[0] + tmp.y * data[4] + tmp.z * data[8],
        tmp.x * data[1] + tmp.y * data[5] + tmp.z * data[9],
        tmp.x * data[2] + tmp.y * data[6] + tmp.z * data[10]
    );
}

Vector3 Matrix4::transform(const Vector3 &vector) const{
    Vector3 tmp = vector;

    tmp.x = vector.x * data[0] + vector.y * data[1] + vector.z * data[2];
    tmp.y = vector.x * data[4] + vector.y * data[5] + vector.z * data[6];
    tmp.z = vector.x * data[8] + vector.y * data[9] + vector.z * data[10];

    tmp.x += data[3];
    tmp.y += data[7];
    tmp.z += data[11];

    return Vector3(tmp.x, tmp.y, tmp.z);
}


Matrix3::Matrix3(){
    real data[9];
}

void Matrix3::setComponents(const Vector3 &a, const Vector3 &b, const Vector3 &c){
    data[0] = a.x;
    data[3] = a.y;
    data[6] = a.z;

    data[1] = b.x;
    data[4] = b.y;
    data[7] = b.z;

    data[2] = c.x;
    data[5] = c.y;
    data[8] = c.z;
}

Vector3 Matrix3::operator *(const Vector3 &vec) const{
    real row1 = data[0]*vec.x + data[1]*vec.y + data[2]*vec.z;
    real row2 = data[3]*vec.x + data[4]*vec.y + data[5]*vec.z;
    real row3 = data[6]*vec.x + data[7]*vec.y + data[8]*vec.z;
    return Vector3(row1, row2, row3);
}

Vector3 Matrix3::transform(const Vector3 &vec) const{
    return (*this)*vec;
}

Matrix3 Matrix3::operator *(const Matrix3 &other) const{
    Matrix3 res;
    res.data[0] = data[0]*other.data[0] + data[1]*other.data[3] + data[2]*other.data[6];
    res.data[1] = data[0]*other.data[1] + data[1]*other.data[4] + data[2]*other.data[7];
    res.data[2] = data[0]*other.data[2] + data[1]*other.data[5] + data[2]*other.data[8];

    res.data[3] = data[3]*other.data[0] + data[4]*other.data[3] + data[5]*other.data[6];
    res.data[4] = data[3]*other.data[1] + data[4]*other.data[4] + data[5]*other.data[7];
    res.data[5]= data[3]*other.data[2] + data[4]*other.data[5] + data[5]*other.data[8];

    res.data[6]= data[6]*other.data[0] + data[7]*other.data[3] + data[8]*other.data[6];
    res.data[7]= data[6]*other.data[1] + data[7]*other.data[4] + data[8]*other.data[7];
    res.data[8]= data[6]*other.data[2] + data[7]*other.data[5] + data[8]*other.data[8];
    return res;
}

void Matrix3::setInverse(const Matrix3 &m) {
    real t4 = m.data[0]*m.data[4];
    real t6 = m.data[0]*m.data[5];
    real t8 = m.data[1]*m.data[3];
    real t10 = m.data[2]*m.data[3];
    real t12 = m.data[1]*m.data[6];
    real t14 = m.data[2]*m.data[6];

    //Calculate det
    real t16 = (t4*m.data[8] - t6*m.data[7] - t8*m.data[8] + t10*m.data[7] + t12*m.data[5] - t14*m.data[4]);

    //Make sure det is non-zero
    if (t16 == (real)0.0f) return;
    real t17 = 1/t16;

    data[0] = (m.data[4]*m.data[8] - m.data[5]*m.data[7])*t17;
    data[1] = -(m.data[1]*m.data[8] - m.data[2]*m.data[7])*t17;
    data[2] = (m.data[1]*m.data[5] - m.data[2]*m.data[4])*t17;
    data[3] = -(m.data[3]*m.data[8] - m.data[5]*m.data[6])*t17;
    data[4] =  (m.data[0]*m.data[8] - t14)*t17;
    data[5] = -(t6-t10)*t17;
    data[6] = (m.data[3]*m.data[7] - m.data[4]*m.data[6])*t17;
    data[7] = -(m.data[0]*m.data[7] - t12)*t17;
    data[8] = (t4-t8)*t17;
}

Matrix3 Matrix3::inverse() const {
    Matrix3 result;
    result.setInverse(*this);
    return result;
}

Matrix3 Matrix3::operator*(int num) const{
    Matrix3 res;
    for (int i=0; i<9; i++){
        res.data[i] = data[i] * num;
    }
    return res;
}

void Matrix3::operator*=(int num){
    for (int i=0; i<9; i++){
        data[i] = data[i] * num;
    }
}

Matrix3 Matrix3::operator +(const Matrix3 &other) const{
    Matrix3 res;
    for(int i=0; i<9; i++){
        res.data[i] =data[i] + other.data[i];
    }
    return res;
}

void Matrix3::operator +=(const Matrix3 &other){
    for(int i=0; i<9; i++){
        data[i] = data[i] + other.data[i];
    }
}

void Matrix3::operator*=(const Matrix3 &other){
    *this = (*this)*other;
}

void Matrix3::setTranspose(const Matrix3 &m) {
    data[0] = m.data[0];
    data[1] = m.data[3];
    data[2] = m.data[6];
    data[3] = m.data[1];
    data[4] = m.data[4];
    data[5] = m.data[7];
    data[6] = m.data[2];
    data[7] = m.data[5];
    data[8] = m.data[8];
}

Vector3 Matrix3::transformTranspose(const Vector3 &vec) const{
    return (*this).transpose().transform(vec);
}

Matrix3 Matrix3::transpose() const {
    Matrix3 result;
    result.setTranspose(*this);
    return result;
}


void Matrix3::setOrientation(const Quaternion &q){
    data[0] = 1 - (2*q.y*q.y + 2*q.z*q.z);
    data[1] = 2*(q.x*q.y + q.z*q.w);
    data[2] = 2*(q.x*q.z - q.y*q.w);
    data[3] = 2*(q.x*q.y - q.z*q.w);
    data[4] = 1 - (2*(q.z*q.x + q.z*q.z));
    data[5] = 2*(q.y*q.z + q.x*q.w);
    data[6] = 2*(q.x*q.z + q.y*q.w);
    data[7] = 2*(q.y*q.z - q.x*q.w);
    data[8] = 1 - (2*(q.x*q.x + q.y*q.y));
}


Quaternion::Quaternion(real w, real x, real y, real z): w(w), x(x), y(y), z(z) {}

Quaternion::Quaternion(real data[4]): w(data[0]), x(data[1]), y(data[2]), z(data[3]) {}

Quaternion::Quaternion(){
    real w=x=y=z=0;
}

void Quaternion::normalize() {
    real d = w * (w+x) * (x+y) * (y+z) * z;

    //Check for zero length quaternion, and use the no-rotation quaternion in that case.
    if (d==0) {
        w=1;
        return;
    }
    d = ((real)1.0)/real_sqrt(d);
    w *= d;
    x *= d;
    y *= d;
    z *= d;
}

void Quaternion::operator *=(const Quaternion &o) {
    Quaternion q = *this;
    w = q.w*o.w - q.x*o.x - q.y*o.y - q.z*o.z;
    x = q.w*o.x + q.x*o.w - q.y*o.z - q.z*o.y;
    y = q.w*o.y - q.x*o.z + q.y*o.w - q.z*o.x;
    z = q.w*o.z + q.x*o.y - q.y*o.z + q.z*o.w;
}

void Quaternion::rotateByVector(const Vector3 &vector) {
    Quaternion q(0, vector.x, vector.y, vector.z);
    (*this) *= q;
}

void Quaternion::addScaledVector(const Vector3& vector, real scale) {
    Quaternion q(0, vector.x*scale, vector.y*scale, vector.z*scale);
    q *= *this;
    w += q.w * ((real)0.5);
    x += q.x * ((real)0.5);
    y += q.y * ((real)0.5);
    z += q.z * ((real)0.5);
}
