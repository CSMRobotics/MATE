#ifndef VECTOR3_HPP
#define VECTOR3_HPP

#include <math.h>
#include <iostream>

namespace csmutil {

// templates are (not) fun :)
template<typename T>
class Vector3 {
public:
    Vector3() = default;
    Vector3(T i, T j, T k) {
        m_i = i;
        m_j = j;
        m_k = k;
    };
    // allow static_cast<T>(Vector3<Y>) to work. NOTE: WILL NOT WORK FOR TYPES THAT FAIL static_cast<T>(Y);
    template<typename Y>
    explicit Vector3(const Vector3<Y>& vector) {
        m_i = static_cast<T>(vector.m_i);
        m_j = static_cast<T>(vector.m_j);
        m_k = static_cast<T>(vector.m_k);
    };
    ~Vector3() = default;

    T getI() {return m_i;};
    T getJ() {return m_j;};
    T getK() {return m_k;};
    T getMagnitude() {return sqrt(m_i * m_i + m_j * m_j + m_k * m_k);};
    
    // set components as three distinct parameters
    void setComponents(T i, T j, T k) {
        m_i = i;
        m_j = j;
        m_k = k;
    };

    // set a vector equal to an array[3] of type T
    void setComponents(const T* ijk) {
        m_i = ijk[0];
        m_j = ijk[1];
        m_k = ijk[2];
    };

    /* convert this vector to a unit vector
     * NOTE: implementation does not make sense for vectors of types != float/double
    */
    Vector3<T> toUnitVector() {
        T mag = getMagnitude();
        return Vector3<T>(static_cast<T>(m_i / mag),
                          static_cast<T>(m_j / mag),
                          static_cast<T>(m_k / mag));
    };

    // scalar = Vector x Vector
    Vector3<T> cross(const Vector3<T>& other) {
        return Vector3<T>((this->m_j * other.m_k) - (this->m_k * other.m_j),
                          (this->m_k * other.m_i) - (this->m_i * other.m_k),
                          (this->m_i * other.m_j) - (this->m_j * other.m_i));
    };

    // scalar = Vector (dot) Vector
    T dot(const Vector3<T>& other) {
        return ((this->m_i * other.m_i) + (this->m_j * other.m_j) + (this->m_k * other.m_k));
    };

    // vector == vector component wise
    friend bool operator==(const Vector3<T>& lhs, const Vector3<T>& rhs) {
        return (lhs.m_i == rhs.m_i) && (lhs.m_j == rhs.m_j) && (lhs.m_k == rhs.m_k);
    };

    // allow vector = vector
    Vector3<T>& operator=(const Vector3<T>& vec) {
        if(&vec == this)
            return *this;
        
        this->m_i = vec.m_i;
        this->m_j = vec.m_j;
        this->m_k = vec.m_k;

        return *this;
    };

    // component wise addition
    friend Vector3<T> operator+(Vector3<T> lhs, const Vector3<T>& rhs) {
        return Vector3<T>(lhs.m_i + rhs.m_i, lhs.m_j + rhs.m_j, lhs.m_k + rhs.m_k);
    };
    // compound assignment
    Vector3<T>& operator+=(const Vector3<T>& vec) {
        this->m_i += vec.m_i;
        this->m_j += vec.m_j;
        this->m_k += vec.m_k;
        return *this;
    };

    // component wise subtraction
    friend Vector3<T> operator-(Vector3<T> lhs, const Vector3<T>& rhs) {
        return Vector3<T>(lhs.m_i - rhs.m_i, lhs.m_j - rhs.m_j, lhs.m_k - rhs.m_k);
    };
    Vector3<T>& operator-=(const Vector3<T>& vec) {
        this->m_i -= vec.m_i;
        this->m_j -= vec.m_j;
        this->m_k -= vec.m_k;
        return *this;
    };

    // scalar multiplication
    template<typename V>
    friend Vector3<T> operator*(Vector3<T> vec, V scalar) {
        return Vector3<T>(static_cast<T>(vec.m_i * scalar), static_cast<T>(vec.m_j * scalar), static_cast<T>(vec.m_k * scalar));
    };

    // scalar compound multiplication
    Vector3<T>& operator*=(T scalar) {
        this->m_i *= scalar;
        this->m_j *= scalar;
        this->m_k *= scalar;
        return *this;
    };

    // scalar division
    template<typename V>
    friend Vector3<T> operator/(Vector3<T> vec, V scalar) {
        return Vector3<T>(static_cast<T>(vec.m_i / scalar), static_cast<T>(vec.m_j / scalar), static_cast<T>(vec.m_k / scalar));
    };

    // scalar compound division
    Vector3<T>& operator/=(T scalar) {
        this->m_i /= scalar;
        this->m_j /= scalar;
        this->m_k /= scalar;
        return *this;
    };
    
    // allow indexing this object
    T operator[](unsigned char idx) const {
        switch(idx) {
        case 0:
            return m_i;
        case 1:
            return m_j;
        case 2:
            return m_k;
        }

        return 0;
    }

    friend std::ostream& operator<<(std::ostream& os, const Vector3<T>& vector) {
        os << '<' << vector.m_i << ", "
                  << vector.m_j << ", "
                  << vector.m_k << '>';
        return os;
    };
private:
    T m_i = 0;
    T m_j = 0;
    T m_k = 0;
};

// heres some handy ones
typedef Vector3<int> Vector3i;
typedef Vector3<float> Vector3f;
typedef Vector3<double> Vector3d;

};

#endif // VECTOR3_HPP