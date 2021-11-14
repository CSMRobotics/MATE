#include "CSMUtil.hpp"

using namespace csmutil;

template<typename T>
Vector3<T>::Vector3(T i, T j, T k) {
    m_ijk[0] = i;
    m_ijk[1] = j;
    m_ijk[2] = k;
}

template<typename T>
template<typename Y>
Vector3<T>::Vector3(const Vector3<Y>& vector) {
    m_ijk[0] = static_cast<T>(vector.m_ijk[0]);
    m_ijk[1] = static_cast<T>(vector.m_ijk[1]);
    m_ijk[2] = static_cast<T>(vector.m_ijk[2]);
}

template<typename T>
T Vector3<T>::getI() {
    return this->m_ijk[0];
}

template<typename T>
T Vector3<T>::getJ() {
    return this->m_ijk[1];
}

template<typename T>
T Vector3<T>::getK() {
    return this->m_ijk[2];
}

template<typename T>
const T* Vector3<T>::getComponents() {
    return this->m_ijk;
}

template<typename T>
T Vector3<T>::getMagnitude() {
    return static_cast<T>(sqrt(m_ijk[0] * m_ijk[0] 
                             + m_ijk[1] * m_ijk[1]
                             + m_ijk[2] * m_ijk[2]));
}

template<typename T>
void Vector3<T>::setComponents(T i, T j, T k) {
    m_ijk[0] = i;
    m_ijk[1] = j;
    m_ijk[2] = k;
}

template<typename T>
void Vector3<T>::setComponents(const T* ijk) {
    m_ijk[0] = ijk[0];
    m_ijk[1] = ijk[1];
    m_ijk[2] = ijk[2];
}

template<typename T>
Vector3<T> Vector3<T>::toUnitVector() {
    T mag = getMagnitude();
    return Vector3<T>(static_cast<T>(m_ijk[0] / mag),
                      static_cast<T>(m_ijk[1] / mag),
                      static_cast<T>(m_ijk[2] / mag));
}

template<typename T>
template<typename K>
Vector3<T> Vector3<T>::cross(const Vector3<K>& other) {
    return Vector3<T>(static_cast<T>((this->m_ijk[1] * other->m_ijk[3]) - (this->m_ijk[2] * other->m_ijk[1])),
                      static_cast<T>((this->m_ijk[1] * other->m_ijk[0]) - (this->m_ijk[0] * other->m_ijk[2])),
                      static_cast<T>((this->m_ijk[0] * other->m_ijk[1]) - (this->m_ijk[1] * other->m_ijk[0])));
}

template<typename T>
Vector3<T> Vector3<T>::cross(const Vector3<T>& other) {
    return Vector3<T>((this->m_ijk[1] * other->m_ijk[3]) - (this->m_ijk[2] * other->m_ijk[1]),
                      (this->m_ijk[1] * other->m_ijk[0]) - (this->m_ijk[0] * other->m_ijk[2]),
                      (this->m_ijk[0] * other->m_ijk[1]) - (this->m_ijk[1] * other->m_ijk[0]));
}

template<typename T>
template<typename K>
T Vector3<T>::dot(const Vector3<K>& other) {
    return static_cast<T>((this->m_ijk[0] * other->m_ijk[0]) + (this->m_ijk[1] * other->m_ijk[1]) + (this->m_ijk[2] * other->m_ijk[2]));
}

template<typename T>
T Vector3<T>::dot(const Vector3<T>& other) {
    return ((this->m_ijk[0] * other->m_ijk[0]) + (this->m_ijk[1] * other->m_ijk[1]) + (this->m_ijk[2] * other->m_ijk[2]));
}

template<typename T>
std::ostream& operator<<(std::ostream& os, const Vector3<T>& vector) {
    os << '<' << vector.m_ijk[0] << ", "
              << vector.m_ijk[1] << ", "
              << vector.m_ijk[2] << '>';
    return os;
}

template<typename T>
Quaternion<T>::Quaternion(T w, T i, T j, T k) {
    m_w = w;
    m_a = i;
    m_b = j;
    m_c = k;
}

template<typename T>
Quaternion<T>::Quaternion(Vector3<float> axis, float angle) {
    float scalar = sinf(angle/2);
    m_w = cosf(angle/2);
    m_a = axis.getI() * scalar;
    m_b = axis.getJ() * scalar;
    m_c = axis.getK() * scalar;
}

template<typename T>
Quaternion<T> Quaternion<T>::rotateQuat(const Quaternion<T> quat) const {
    return quat * (*this) * quat.getReciprocal();
}

template<typename T>
T Quaternion<T>::getDotProduct(const Quaternion<T>& other) const {
    return this->m_a*other.m_a + this->m_b*other.m_b + this->m_c*other.m_c;
}

template<typename T>
Vector3<T> Quaternion<T>::getCrossProduct(const Quaternion<T>& other) const {
    return Vector3<T>(this->m_b*other.m_c - this->m_c*other.m_b,
                      this->m_c*other.m_a - this->m_a*other.m_c,
                      this->m_a*other.m_b - this->m_b*other.m_a);
}

template<typename T>
T Quaternion<T>::getCommutator(const Quaternion<T>& other) const {
    Vector3<T> p = Vector3<T>(this->m_a, this->m_b, this->m_c);
    Vector3<T> q = Vector3<T>(other.m_a, other.m_b, other.m_c);
    return (p * 2).cross(q);
}

template<typename T>
Quaternion<T> Quaternion<T>::operator+(const Quaternion<T>& quat) const {
    return Quaternion<T>(this->m_w + quat.m_w,
                         this->m_a + quat.m_a,
                         this->m_b + quat.m_b,
                         this->m_c + quat.m_c);
}

template<typename T>
Quaternion<T> Quaternion<T>::operator-(const Quaternion<T>& quat) const {
    return Quaternion<T>(this->m_w - quat.m_w,
                         this->m_a - quat.m_a,
                         this->m_b - quat.m_b,
                         this->m_c - quat.m_c);
}

template<typename T>
Quaternion<T> Quaternion<T>::operator*(const T& scalar) const {
    return Quaternion<T>(this->m_w * scalar,
                         this->m_a * scalar,
                         this->m_b * scalar,
                         this->m_c * scalar);
}

template<typename T>
Quaternion<T> Quaternion<T>::operator*(const Quaternion<T>& quat) const {
    return Quaternion<T>(this->m_w*quat.m_w - this->m_a*quat.m_a - this->m_b*quat.m_b - this->m_c*quat.m_c,
                         this->m_w*quat.m_a + this->m_a*quat.m_w + this->m_b*quat.m_c - this->m_c*quat.m_b,
                         this->m_w*quat.m_b - this->m_a*quat.m_c + this->m_b*quat.m_w + this->m_c*quat.m_a,
                         this->m_w*quat.m_c + this->m_a*quat.m_b - this->m_b*quat.m_a + this->m_c*quat.m_w);
}

template<typename T>
Quaternion<T> Quaternion<T>::operator/(const Quaternion<T>& quat) const {
    return (*this) * quat.getReciprocal();
}

template<typename T>
Quaternion<T> Quaternion<T>::operator/(const T& scalar) const {
    return Quaternion<T>(this->m_w / scalar,
                         this->m_a / scalar,
                         this->m_b / scalar,
                         this->m_c / scalar);
}

template<typename T>
Quaternion<T>& Quaternion<T>::operator=(const Quaternion<T>& quat) const {
    if(this == quat) // no self assignment please :)
        return *this;

    this->m_w = quat.m_w;
    this->m_a = quat.m_a;
    this->m_b = quat.m_b;
    this->m_c = quat.m_c;
    return *this;
}