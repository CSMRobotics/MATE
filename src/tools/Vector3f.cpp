#include <include/tools/Vector3f.hpp>
#include <math.h>

Vector3f::Vector3f(float i, float j, float k) {
    this->i = i;
    this->j = j;
    this->k = k;
}

Vector3f::~Vector3f() {

}

float Vector3f::getI() {
    return this->i;
}

float Vector3f::getJ() {
    return this->j;
}

float Vector3f::getK() {
    return this->k;
}

float* Vector3f::getComponents() {
    float* toRet = new float[3];
    toRet[0] = this->i;
    toRet[1] = this->j;
    toRet[2] = this->k;
    return toRet;
}

float Vector3f::getMagnitude() {
    return sqrtf(powf(this->i, 2) + powf(this->j, 2) + powf(this->k, 2));
}

Vector3f Vector3f::getUnitVector() {
    float mag = this->getMagnitude();
    return Vector3f(this->i / mag, this->j / mag, this->k / mag);
}

void Vector3f::setComponents(float i, float j, float k) {
    this->i = i;
    this->j = j;
    this->k = k;
}

void Vector3f::setComponents(const float* ijk) {
    this->i = ijk[0];
    this->j = ijk[1];
    this->k = ijk[2];
}

Vector3f Vector3f::crossProduct(const Vector3f &other) {
    return Vector3f((this->j * other.k) - (this->k * other.j), (this->j * other.i) - (this-> i * other.k), (this->i * other.j) - (this->j * other.i));
}

float Vector3f::dotProduct(const Vector3f &other) {
    return ((this->i * other.i) + (this->j * other.j) + (this->k * other.k));
}