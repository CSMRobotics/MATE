#ifndef CSMUTIL_HPP
#define CSMUTIL_HPP

#include <bitset>
#include <cmath>
#include <iostream>

namespace csmutil{

static int imap(float value, float from_min, float from_max, int to_min, int to_max){
    return ((value - from_min) * ((to_max - to_min) / (from_max - from_min)) + to_min);
}

template<typename R, typename T, typename F>
static R map(F value, F from_min, F from_max, T to_min, T to_max) {
    return (R)((value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min);
}

// templates are (not) fun :)
template<typename T>
class Vector3 {
public:
    Vector3() = default;
    Vector3(T i, T j, T k);
    // allow static_cast<T>(Vector3<Y>) to work NOTE: WILL NOT WORK FOR TYPES THAT FAIL static_cast<T>(Y);
    template<typename Y>
    explicit Vector3(const Vector3<Y>& vector);
    ~Vector3() = default;

    // get the components as either type T or as pointer to array of type T
    T getI();
    T getJ();
    T getK();
    T getMagnitude();
    
    // set components as either three distinct parameters or as pointer to array of type T
    void setComponents(T i, T j, T k);
    void setComponents(const T* ijk);

    // Vector math
    Vector3<T> toUnitVector();
    template<typename K>
    Vector3<T> cross(const Vector3<K>& other);
    Vector3<T> cross(const Vector3<T>& other);
    template<typename K>
    T dot(const Vector3<K>& other);
    T dot(const Vector3<T>& other);

    // TODO:implement
    // operators (not going to make version for intercomparibility of different types of Vector3s because i dont want to :) ) 
    template<typename U>
    friend bool operator==(const Vector3<U>& lhs, const Vector3<U>& rhs);

    Vector3<T>& operator=(const Vector3<T>& vec);
    template<typename U>
    Vector3<T>& operator=(const Vector3<U>& vec);

    // component wise addition
    template<typename U> // NOTE: i used a different typename here. idk if it works like that
    friend Vector3<U>& operator+(Vector3<U> lhs, const Vector3<U>& rhs);
    // compound assignment
    Vector3<T>& operator+=(const Vector3<T>& vec);

    // component wise subtraction
    template<typename U>
    friend Vector3<U>& operator-(Vector3<U> lhs, const Vector3<U>& rhs);
    Vector3<T>& operator-=(const Vector3<T>& vec);
    
    const T& operator[](unsigned char idx) const;

    // copy and swap operator
    Vector3<T>& operator=(Vector3<T> vec) noexcept;

    /* declare new typename to not step on T
    * and allow cout << Vector3<any>
    */
    template<typename Y>
    friend std::ostream& operator<<(std::ostream& os, const Vector3<Y>& vector);
private:
    T m_i;
    T m_j;
    T m_k;
};

template<typename T>
bool operator==(const Vector3<T>& lhs, const Vector3<T>& rhs);

template<typename T>
Vector3<T>& operator+(Vector3<T> lhs, const Vector3<T>& rhs);
template<typename T>
Vector3<T>& operator-(Vector3<T> lhs, const Vector3<T>& rhs);

template<typename T>
std::ostream& operator<<(std::ostream& os, const Vector3<T>& vector);

// heres some handy ones
typedef Vector3<int> Vector3i;
typedef Vector3<float> Vector3f;
typedef Vector3<double> Vector3d;


template<typename T>
class Quaternion {
public:
    Quaternion() = default;
    Quaternion(T w, T i, T j, T k);
    // NOTE: expects radians
    Quaternion(Vector3f axis, float angle);
    // allow static_cast<T>(Quaternion<Y>) to work NOTE: WILL NOT WORK FOR TYPES THAT FAIL static_cast<T>(Y);
    template<typename Y>
    explicit Quaternion(const Quaternion<Y>& quaternion);

    // get the components as either type T or as pointer to array of type T
    T getW() {return m_w;};
    T getI() {return m_a;};
    T getJ() {return m_b;};
    T getK() {return m_c;};

    // TODO: get euler angles, get axis angle, e.g. finish implementing -> https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Using_quaternions_as_rotations
    T getScalar() {return m_a;};
    Vector3<T> getVector() {return Vector3<T>(m_a, m_b, m_c);};
    Quaternion<T> getConjugate() {return Quaternion<T>(m_w, -m_a, -m_b, -m_c);};
    T getNorm() {return sqrt(m_w*m_w + m_a*m_a + m_b*m_b + m_c*m_c);};
    T getDistanceToQuat(const Quaternion<T> quat) {return (*this - quat).getNorm();};
    Quaternion<T> getAsUnit() {return (*this) / getNorm();}; // aka versor
    Quaternion<T> getReciprocal() {T norm = getNorm();return getConjugate() / (norm * norm);};

    bool isVectorQuat() {return m_w == 0;};
    bool isScalarQuat() {return m_a == 0 && m_b == 0 && m_c == 0;};

    Quaternion<T> rotateQuat(const Quaternion<T> quat) const;

    T getDotProduct(const Quaternion<T>& other) const;
    Vector3<T> getCrossProduct(const Quaternion<T>& other) const;
    T getCommutator(const Quaternion<T>& other) const;

    // component-wise addition
    Quaternion<T> operator+(const Quaternion<T>& quat) const;
    // component-wise subtraction
    Quaternion<T> operator-(const Quaternion<T>& quat) const;
    // scalar multiplication
    Quaternion<T> operator*(const T& scalar) const;
    // hamilton product (not commutative (unless real), but is associative)
    Quaternion<T> operator*(const Quaternion<T>& quat) const;
    // q/p != p/q because p*q^-1 != p^-1*q
    Quaternion<T> operator/(const Quaternion<T>& quat) const;
    // scalar division
    Quaternion<T> operator/(const T& scalar) const;

    // assign this quaternion's values to another
    Quaternion<T>& operator=(const Quaternion<T>& quat) const;

    // TODO: more advanced match functions? exponential, log, and power are very disgusting btw
private:
    T m_w;
    T m_a;
    T m_b;
    T m_c;
};

};

class PIDController {
    public:
        PIDController() = default;
        PIDController(float kp, float ki, float kd);

        void Update(float error, float dt);
        float output;
    private:
        // Gains
        float kp = 0;
        float ki = 0;
        float kd = 0;

        // 
        float error_prior = 0;
        float integral_prior = 0;
};

#endif // CSMUTIL_HPP