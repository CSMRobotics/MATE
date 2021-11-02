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

