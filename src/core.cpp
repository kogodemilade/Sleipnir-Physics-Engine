#include "../include/Sleipnir/core.hpp" //CHANGE TO #include "cyclone/core.hpp"
#include <cmath>

using namespace cyclone;

cyclone::Vector3 cyclone::Vector3::localToWorld(const cyclone::Vector3 &local, const Matrix4 &transform) const{
    return transform.transform(local);
}

cyclone::Vector3 cyclone::Vector3::worldToLocal(const cyclone::Vector3 &world, const Matrix4 &transform) const{
    return transform.transformInverse(world);
}

void cyclone::Vector3::setLocalVec(const Matrix4 &transform) {
    cyclone::Vector3 tmp = worldToLocal(*this, transform);
    x = tmp.x;
    y = tmp.y;
    z = tmp.z;
}

void cyclone::Vector3::setWorldVec(const Matrix4 &transform) {
    cyclone::Vector3 tmp = localToWorld(*this, transform);
    x = tmp.x;
    y = tmp.y;
    z = tmp.z;
}

cyclone::Vector3 cyclone::Vector3::localToWorldDirn(const cyclone::Vector3 &local, const Matrix4 &transform) const {
    return transform.transformDirection(local);
}

cyclone::Vector3 cyclone::Vector3::worldToLocalDirn(const cyclone::Vector3 &world, const Matrix4 &transform) const {
    return transform.transformInvDir(world);
}

void cyclone::Vector3::setLocalVecDirn(const Matrix4 &transform) {
    cyclone::Vector3 tmp = worldToLocalDirn(*this, transform);
    x = tmp.x;
    y = tmp.y;
    z = tmp.z;
}



void cyclone::Vector3::setWorldVecDirn(const Matrix4 &transform) {
    cyclone::Vector3 tmp = localToWorldDirn(*this, transform);
    x = tmp.x;
    y = tmp.y;
    z = tmp.z;
}

Matrix3 cyclone::Vector3::skewSymmetricMatrix(){
    Matrix3 res;
    res.data[0] = res.data[4] = res.data[8] = 0;
    res.data[1] = -z;
    res.data[2] = y;
    res.data[3] = z;
    res.data[5] = -x;
    res.data[6] = -y;
    res.data[7] = x;
    return res;
}


cyclone::Quaternion cyclone::Vector3::toQuaternion()const{
    cyclone::Vector3 copy = *this;
    // copy.normalize();
    cyclone::real angle = copy.magnitude();

    cyclone::real half = angle*0.5f;
    cyclone::real s = sin(half);

    cyclone::Quaternion q;
    q.w = cos(half);
    q.x = copy.x * s;
    q.y = copy.y * s;
    q.z = copy.z * s;
    q.normalize();
    return q;
}

glm::vec3 cyclone::Vector3::toGlm() const{
    return glm::vec3(x, y, z);
}





Matrix4::Matrix4(){
    data[12];
}

cyclone::Vector3 Matrix4::transformDirection(const cyclone::Vector3 &vector) const {
    return cyclone::Vector3(
            vector.x*data[0] +vector.y * data[1] + vector.z *data[2],
            vector.x*data[4] +vector.y * data[5] + vector.z *data[6],
            vector.x*data[8] +vector.y * data[9] + vector.z *data[10]
        );
    }

cyclone::Vector3 Matrix4::transformInvDir(const cyclone::Vector3 &vector) const {
    return cyclone::Vector3(
    vector.x * data[0] + vector.y * data[4] + vector.z * data[8],
    vector.x * data[1] + vector.y * data[5] + vector.z * data[9],
    vector.x * data[2] + vector.y * data[6] + vector.z * data[10]
    );
}

cyclone::real Matrix4::getDeterminant() const {
    float t1 = data[0] * data[5] * data[10];
float t2 = data[0] * data[6] * data[9];
float t3 = data[1] * data[4] * data[10];
float t4 = data[1] * data[6] * data[8];
float t5 = data[2] * data[4] * data[9];
float t6 = data[2] * data[5] * data[8];

return t1 - t2 - t3 + t4 + t5 - t6;
}

cyclone::Vector3 Matrix4::getAxisVector(int i) const {
    return cyclone::Vector3(data[i], data[i+4], data[i+8]);
}

void Matrix4::setInverse(const Matrix4 &m) {
    //Make sure det is non-zero
    cyclone::real det = getDeterminant();
    if (det == 0) return;
    det = ((cyclone::real)1.0)/det;

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

cyclone::Vector3 Matrix4::operator*(const cyclone::Vector3 &vec) const {
    return cyclone::Vector3(
        (vec.x*data[0]+vec.y*data[1]+vec.z*data[2]) + data[3],
        (vec.x*data[4]+vec.y*data[5]+vec.z*data[6]) + data[7],
        (vec.x*data[8]+vec.y*data[9]+vec.z*data[10]) + data[11]
    );
}

void Matrix4::setOrientAndPos(const cyclone::Quaternion &q, const cyclone::Vector3 &pos){
    data[0] = 1 - (2*q.y*q.y + 2*q.z*q.z);
    data[1] = 2*(q.x*q.y + q.z*q.w);
    data[2] = 2*(q.x*q.z - q.y*q.w);
    data[3] = pos.x;

    data[4] = 2*(q.x*q.y - q.z*q.w);
    data[5] = 1 - 2*(q.x*q.x + q.z*q.z);
    data[6] = 2*(q.y*q.z + q.x*q.w);
    data[7] = pos.y;

    data[8] = 2*(q.x*q.z + q.y*q.w);
    data[9] = 2*(q.y*q.z - q.x*q.w);
    data[10] = 1 - (2*(q.x*q.x + q.y*q.y));
    data[11] = pos.z;
}

void Matrix4::setZero(){
    for (int i = 0; i<12; i++){
        data[i] = 0;
    }
}

void Matrix4::setOrientAndPos(const Matrix3 &rot, const cyclone::Vector3 &pos){
    data[0] = rot.data[0];
    data[1] = rot.data[1];
    data[2] = rot.data[2];
    data[3] = pos.x;

    data[4] = rot.data[3];
    data[5] = rot.data[4];
    data[6] = rot.data[5];
    data[7] = pos.y;

    data[8] = rot.data[6];
    data[9] = rot.data[7];
    data[10] = rot.data[8];
    data[11] = pos.z;
}

cyclone::Vector3 Matrix4::transformInverse(const cyclone::Vector3 &vector) const{
    cyclone::Vector3 tmp = vector;
    tmp.x -= data[3];
    tmp.y -= data[7];
    tmp.z -= data[11];

    return cyclone::Vector3(
        tmp.x * data[0] + tmp.y * data[4] + tmp.z * data[8],
        tmp.x * data[1] + tmp.y * data[5] + tmp.z * data[9],
        tmp.x * data[2] + tmp.y * data[6] + tmp.z * data[10]
    );
}

cyclone::Vector3 Matrix4::transform(const cyclone::Vector3 &vector) const{
    cyclone::Vector3 tmp = vector;

    tmp.x = vector.x * data[0] + vector.y * data[1] + vector.z * data[2];
    tmp.y = vector.x * data[4] + vector.y * data[5] + vector.z * data[6];
    tmp.z = vector.x * data[8] + vector.y * data[9] + vector.z * data[10];

    tmp.x += data[3];
    tmp.y += data[7];
    tmp.z += data[11];

    return cyclone::Vector3(tmp.x, tmp.y, tmp.z);
}

glm::mat4 Matrix4::toGlm() const {
    glm::mat4 result(1.0f); // initializes last row/column correctly

    // Cyclone: row-major 4x3 data stored as 12 floats
    // GLM: column-major 4x4

    for (int r = 0; r < 3; r++) {          // only 3 valid rows
        for (int c = 0; c < 4; c++) {      // 4 columns (3 rotation + translation)
            result[c][r] = data[r * 4 + c];
        }
    }

    // Last row should be (0, 0, 0, 1)
    result[0][3] = 0.0f;
    result[1][3] = 0.0f;
    result[2][3] = 0.0f;
    result[3][3] = 1.0f;

    return result;
}

Matrix4 Matrix4::operator+(const Matrix4 &o) const{
    Matrix4 res;
    for (int i =0; i < 12; i++){
        res.data[i] = o.data[i] + data[i];
    }
    return res;
}

Matrix4 Matrix4::operator+(const cyclone::Vector3 &vec) const{
    Matrix4 res;
    for (int i=0; i < 12; i++){
        res.data[i] = data[i];

        if (i==3) res.data[i] += vec[0];
        if (i==7) res.data[i] += vec[1];
        if (i==11) res.data[i] += vec[2];
    }
    return res;
}








Matrix3::Matrix3(){
    data[9];
}

Matrix3::Matrix3(cyclone::real _data[9]){
    for (size_t i=0; i < 9; i++){
        data[i] = _data[i];
    }
}

void Matrix3::setComponents(const cyclone::Vector3 &a, const cyclone::Vector3 &b, const cyclone::Vector3 &c){
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

void Matrix3::setDiagonal(cyclone::real a, cyclone::real b, cyclone::real c){
    for(int i=0; i<9; i++){
        data[i] = 0;
    }
    data[0] = a;
    data[4] = b;
    data[8] = c;
   }

void Matrix3::identityMatrix() {
    for (int i = 0; i < 9; ++i) data[i] = 0;
    data[0] = data[4] = data[8] = 1;
}

void Matrix3::setZero(){
    for (int i = 0; i<9; i++){
        data[i] = 0;
    }
}

cyclone::Vector3 Matrix3::operator *(const cyclone::Vector3 &vec) const{
    cyclone::real row1 = data[0]*vec.x + data[1]*vec.y + data[2]*vec.z;
    cyclone::real row2 = data[3]*vec.x + data[4]*vec.y + data[5]*vec.z;
    cyclone::real row3 = data[6]*vec.x + data[7]*vec.y + data[8]*vec.z;
    return cyclone::Vector3(row1, row2, row3);
}

cyclone::Vector3 Matrix3::transform(const cyclone::Vector3 &vec) const{
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
    cyclone::real t4 = m.data[0]*m.data[4];
    cyclone::real t6 = m.data[0]*m.data[5];
    cyclone::real t8 = m.data[1]*m.data[3];
    cyclone::real t10 = m.data[2]*m.data[3];
    cyclone::real t12 = m.data[1]*m.data[6];
    cyclone::real t14 = m.data[2]*m.data[6];

    //Calculate det
    cyclone::real t16 = (t4*m.data[8] - t6*m.data[7] - t8*m.data[8] + t10*m.data[7] + t12*m.data[5] - t14*m.data[4]);

    //Make sure det is non-zero
    if (fabs(t16) < 1e-9) return;
    cyclone::real t17 = 1/t16;

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

Matrix3 Matrix3::operator*(cyclone::real num) const{
    Matrix3 res;
    for (int i=0; i<9; i++){
        res.data[i] = data[i] * num;
    }
    return res;
}

void Matrix3::operator*=(cyclone::real num){
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

cyclone::Vector3 Matrix3::transformTranspose(const cyclone::Vector3 &vec) const{
    return (*this).transpose().transform(vec);
}

Matrix3 Matrix3::transpose() const {
    Matrix3 result;
    result.setTranspose(*this);
    return result;
}


void Matrix3::setOrientation(const cyclone::Quaternion &q){
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

glm::mat3 Matrix3::toGlm() const{
    glm::mat3 out(1.0f);

    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            out[c][r] = data[r * 3 + c]; 
        }
    }

    return out;
}






cyclone::Quaternion::Quaternion(cyclone::real w, cyclone::real x, cyclone::real y, cyclone::real z): w(w), x(x), y(y), z(z) {}

cyclone::Quaternion::Quaternion(cyclone::real data[4]): w(data[0]), x(data[1]), y(data[2]), z(data[3]) {}

cyclone::Quaternion::Quaternion(){
    w=x=y=z=0;
}

void cyclone::Quaternion::normalize() {
    cyclone::real d = w*w + x*x + y*y + z*z;

    //Check for zero length cyclone::Quaternion, and use the no-rotation cyclone::Quaternion in that case.
    if (d < 0.0001){
        w=1;
        x=y=z=0;
        return;
    }
    d = ((cyclone::real)1.0)/cyclone::real_sqrt(d);
    w *= d;
    x *= d;
    y *= d;
    z *= d;
}

void cyclone::Quaternion::operator *=(const cyclone::Quaternion &o) {
    cyclone::Quaternion q = *this;
    w = q.w*o.w - q.x*o.x - q.y*o.y - q.z*o.z;
    x = q.w*o.x + q.x*o.w + q.y*o.z - q.z*o.y;
    y = q.w*o.y - q.x*o.z + q.y*o.w + q.z*o.x;
    z = q.w*o.z + q.x*o.y - q.y*o.x + q.z*o.w;
}

cyclone::Quaternion cyclone::Quaternion::operator *(const cyclone::Quaternion &o)const {
    cyclone::Quaternion q = *this;
    cyclone::Quaternion res;
    res.w = q.w*o.w - q.x*o.x - q.y*o.y - q.z*o.z;
    res.x = w*o.x + x*o.w + y*o.z - z*o.y;
    res.y = w*o.y - x*o.z + y*o.w + z*o.x;
    res.z = w*o.z + x*o.y - y*o.x + z*o.w;

    return res;
}

void cyclone::Quaternion::rotateByVector(const cyclone::Vector3 &vec) {
cyclone::real angle = sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);

if (angle < 1e-6) return;

cyclone::real half = angle*0.5f;
cyclone::real s = sin(half);
cyclone::real c=cos(half);
}

void cyclone::Quaternion::addScaledVector(const cyclone::Vector3& vector, cyclone::real scale) {
    cyclone::Quaternion q(0, vector.x*scale, vector.y*scale, vector.z*scale);
    q *= *this;
    w += q.w * ((cyclone::real)0.5);
    x += q.x * ((cyclone::real)0.5);
    y += q.y * ((cyclone::real)0.5);
    z += q.z * ((cyclone::real)0.5);
}

cyclone::Vector3 cyclone::Quaternion::toEulerAngles() const{
    cyclone::real w_ = w;
    cyclone::real x_ = x;
    cyclone::real y_ = y;
    cyclone::real z_ = z;

    //Roll (x-axis rotation)
    cyclone::real sinr = 2.0 * (w * x+y * z);
    cyclone::real cosr = 1.0 - 2.0 * (x*x + y*y);
    cyclone::real roll = std::atan2(sinr, cosr);

    //Pitch (y-axis rotation)
    cyclone::real sinp = 2.0 * (w*y - z*x);
    cyclone::real pitch;
    if(std::abs(sinp) >= 1.0){
        pitch = std::copysign(0.5*cyclone::real_pi, sinp);
    } else{
        pitch = std::asin(sinp);
    }

    //Yaw (z-axis rotation)
    cyclone::real siny = 2.0*(w*z + x*y);
    cyclone::real cosy = 1.0 - 2.0*(y*y + z*z);
    cyclone::real yaw = std::atan2(siny, cosy);

    return cyclone::Vector3(roll, pitch, yaw);

}

void cyclone::Quaternion::rotate(const cyclone::Quaternion &other){
    cyclone::Quaternion q = other*(*this);
    *this = q;
}

Matrix3 cyclone::Quaternion::toMatrix() const{
    cyclone::real x_ = x;
    cyclone::real y_ = y;
    cyclone::real z_ = z;
    cyclone::real w_ = w;

    cyclone::real x2 = x_ + x_;
    cyclone::real y2 = y_ + y_;
    cyclone::real z2 = z_ + z_;

    cyclone::real xx = x_ * x2;
    cyclone::real yy = y_ * y2;
    cyclone::real zz = z_ * z2;
    cyclone::real xy = x_ * y2;
    cyclone::real xz = x_ * z2;
    cyclone::real yz = y_ * z2;
    cyclone::real wx = w_ * x2;
    cyclone::real wy = w_ * y2;
    cyclone::real wz = w_ * z2;

    cyclone::real data[9] = {
        1.0f - (yy + zz), xy - wz,       xz + wy,
        xy + wz,         1.0f - (xx + zz), yz - wx,
        xz - wy,         yz + wx,        1.0f - (xx + yy)
    };
    return Matrix3(
        data
    );
}

glm::quat cyclone::Quaternion::toGlm()const{
    return glm::quat(w, x, y, z);
}