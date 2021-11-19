#ifndef CSMUTIL_HPP
#define CSMUTIL_HPP

#include <cmath>
#include <iostream>
#include <algorithm>

namespace csmutil{

static int imap(float value, float from_min, float from_max, int to_min, int to_max){
    return ((value - from_min) * ((to_max - to_min) / (from_max - from_min)) + to_min);
}

template<typename R, typename T, typename F>
static R map(F value, F from_min, F from_max, T to_min, T to_max) {
    return (R)((value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min);
}

// TODO: replace quaternion and vector3 implementation with header-only version
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
    template<typename U>
    friend bool operator==(const Vector3<U>& lhs, const Vector3<U>& rhs) {
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
    template<typename U>
    friend Vector3<U> operator+(Vector3<U> lhs, const Vector3<U>& rhs) {
        return Vector3<U>(lhs.m_i + rhs.m_i, lhs.m_j + rhs.m_j, lhs.m_k + rhs.m_k);
    };
    // compound assignment
    Vector3<T>& operator+=(const Vector3<T>& vec) {
        this->m_i += vec.m_i;
        this->m_j += vec.m_j;
        this->m_k += vec.m_k;
        return *this;
    };

    // component wise subtraction
    template<typename U>
    friend Vector3<U> operator-(Vector3<U> lhs, const Vector3<U>& rhs) {
        return Vector3<U>(lhs.m_i - rhs.m_i, lhs.m_j - rhs.m_j, lhs.m_k - rhs.m_k);
    };
    Vector3<T>& operator-=(const Vector3<T>& vec) {
        this->m_i -= vec.m_i;
        this->m_j -= vec.m_j;
        this->m_k -= vec.m_k;
        return *this;
    };

    // scalar multiplication
    template<typename U, typename V>
    friend Vector3<U> operator*(Vector3<U> vec, V scalar) {
        return Vector3<U>(static_cast<U>(vec.m_i * scalar), static_cast<U>(vec.m_j * scalar), static_cast<U>(vec.m_k * scalar));
    };

    template<typename U, typename V>
    friend Vector3<U> operator*(V scalar, Vector3<U> vec) {
        return Vector3<U>(static_cast<U>(vec.m_i * scalar), static_cast<U>(vec.m_j * scalar), static_cast<U>(vec.m_k * scalar));
    };

    // scalar compound multiplication
    Vector3<T>& operator*=(T scalar) {
        this->m_i *= scalar;
        this->m_j *= scalar;
        this->m_k *= scalar;
        return *this;
    };

    // scalar division
    template<typename U, typename V>
    friend Vector3<U> operator/(Vector3<U> vec, V scalar) {
        return Vector3<U>(static_cast<U>(vec.m_i / scalar), static_cast<U>(vec.m_j / scalar), static_cast<U>(vec.m_k / scalar));
    };

    template<typename U, typename V>
    friend Vector3<U> operator/(V scalar, Vector3<U> vec) {
        return Vector3<U>(static_cast<U>(vec.m_i / scalar), static_cast<U>(vec.m_j / scalar), static_cast<U>(vec.m_k / scalar));
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

    /* declare new typename to not step on T
    * and allow cout << Vector3<any>
    */
    template<typename Y>
    friend std::ostream& operator<<(std::ostream& os, const Vector3<Y>& vector) {
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


template<typename T>
class Quaternion {
public:
    Quaternion() = default;
    Quaternion(T w, T i, T j, T k) {
        m_w = w;
        m_i = i;
        m_j = j;
        m_k = k;
    };
    // NOTE: expects radians
    Quaternion(Vector3f axis, float angle) {
        float scalar = sinf(angle/2);
        m_w = cosf(angle/2);
        m_i = axis.getI() * scalar;
        m_j = axis.getJ() * scalar;
        m_k = axis.getK() * scalar;
    };
    // allow static_cast<T>(Quaternion<Y>) to work NOTE: WILL NOT WORK FOR TYPES THAT FAIL static_cast<T>(Y);
    template<typename Y>
    explicit Quaternion(const Quaternion<Y>& quaternion) {
        m_w = static_cast<T>(quaternion.m_w);
        m_i = static_cast<T>(quaternion.m_i);
        m_j = static_cast<T>(quaternion.m_j);
        m_k = static_cast<T>(quaternion.m_k);
    };

    // get the components as either type T or as pointer to array of type T
    T getW() {return m_w;};
    T getI() {return m_i;};
    T getJ() {return m_j;};
    T getK() {return m_k;};

    // TODO: get euler angles, get axis angle, e.g. finish implementing -> https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Using_quaternions_as_rotations
    T getScalar() {return m_i;};
    Vector3<T> getVector() {return Vector3<T>(m_i, m_j, m_k);};
    Quaternion<T> getConjugate() {return Quaternion<T>(m_w, -m_i, -m_j, -m_k);};
    T getNorm() {return sqrt(m_w*m_w + m_i*m_i + m_j*m_j + m_k*m_k);};
    T getDistanceToQuat(const Quaternion<T> quat) {return (*this - quat).getNorm();};
    Quaternion<T> getAsUnit() {return (*this) / getNorm();}; // aka versor
    Quaternion<T> getReciprocal() {T norm = getNorm();return getConjugate() / (norm * norm);};

    bool isVectorQuat() {return m_w == 0;};
    bool isScalarQuat() {return m_i == 0 && m_j == 0 && m_k == 0;};

    Quaternion<T> rotateQuat(const Quaternion<T> quat) const {
        return quat * (*this) * quat.getReciprocal();
    };

    // component-wise addition
    Quaternion<T> operator+(const Quaternion<T>& quat) const {
        return Quaternion<T>(this->m_w + quat.m_w,
                             this->m_i + quat.m_i,
                             this->m_j + quat.m_j,
                             this->m_k + quat.m_k);
    };

    // component-wise subtraction
    Quaternion<T> operator-(const Quaternion<T>& quat) const {
        return Quaternion<T>(this->m_w - quat.m_w,
                             this->m_i - quat.m_i,
                             this->m_j - quat.m_j,
                             this->m_k - quat.m_k);
    };

    // scalar multiplication
    Quaternion<T> operator*(const T& scalar) const {
        return Quaternion<T>(this->m_w * scalar,
                             this->m_i * scalar,
                             this->m_j * scalar,
                             this->m_k * scalar);
    };

    // hamilton product (not commutative (unless real), but is associative)
    Quaternion<T> operator*(const Quaternion<T>& quat) const {
        return Quaternion<T>(this->m_w*quat.m_w - this->m_i*quat.m_i - this->m_j*quat.m_j - this->m_k*quat.m_k,
                             this->m_w*quat.m_i + this->m_i*quat.m_w + this->m_j*quat.m_k - this->m_k*quat.m_j,
                             this->m_w*quat.m_j - this->m_i*quat.m_k + this->m_j*quat.m_w + this->m_k*quat.m_i,
                             this->m_w*quat.m_k + this->m_i*quat.m_j - this->m_j*quat.m_i + this->m_k*quat.m_w);
    };

    /* p * 1/q != 1/q * p because p*q^-1 != p^-1*q
     * This implementation assumes p * 1/q
    */
    Quaternion<T> operator/(const Quaternion<T>& quat) const {
        return (*this) * quat.getReciprocal();
    };

    // scalar division
    Quaternion<T> operator/(const T& scalar) const {
        return Quaternion<T>(this->m_w / scalar,
                             this->m_i / scalar,
                             this->m_j / scalar,
                             this->m_k / scalar);
    };

    // assign this quaternion's values to another
    Quaternion<T>& operator=(const Quaternion<T>& quat) {
        if(this == &quat) // no self assignment please :)
            return *this;

        this->m_w = quat.m_w;
        this->m_i = quat.m_i;
        this->m_j = quat.m_j;
        this->m_k = quat.m_k;
        return *this;
    };

    // compound operators
    Quaternion<T> operator+=(Quaternion<T> quat) {
        this->m_w += quat.m_w;
        this->m_i += quat.m_i;
        this->m_j += quat.m_j;
        this->m_k += quat.m_k;
        return *this;
    };

    Quaternion<T> operator-=(Quaternion<T> quat) {
        this->m_w -= quat.m_w;
        this->m_i -= quat.m_i;
        this->m_j -= quat.m_j;
        this->m_k -= quat.m_k;
        return *this;
    };

    // hamilton product
    Quaternion<T> operator*=(Quaternion<T> quat) {
        this->m_w = this->m_w*quat.m_w - this->m_i*quat.m_i - this->m_j*quat.m_j - this->m_k*quat.m_k;
        this->m_i = this->m_w*quat.m_i + this->m_i*quat.m_w + this->m_j*quat.m_k - this->m_k*quat.m_j;
        this->m_j = this->m_w*quat.m_j - this->m_i*quat.m_k + this->m_j*quat.m_w + this->m_k*quat.m_i;
        this->m_k = this->m_w*quat.m_k + this->m_i*quat.m_j - this->m_j*quat.m_i + this->m_k*quat.m_w;
        return *this;
    };

    // scalar multiplication
    Quaternion<T> operator*=(T scalar) {
        this->m_w *= scalar;
        this->m_i *= scalar;
        this->m_j *= scalar;
        this->m_k *= scalar;
        return *this;
    };

    // assumes p * 1/q
    Quaternion<T> operator/=(Quaternion<T> quat) {
        (*this) = (*this) * quat.getReciprocal();
        return *this;
    };

    // scalar division
    Quaternion<T> operator/=(T scalar) {
        this->m_w /= scalar;
        this->m_i /= scalar;
        this->m_j /= scalar;
        this->m_k /= scalar;
        return *this;
    };


    // TODO: more advanced match functions? exponential, log, and power are very disgusting btw
private:
    T m_w = 0;
    T m_i = 0;
    T m_j = 0;
    T m_k = 0;
};

// few handy quaternions
typedef Quaternion<float> Quaternionf;
typedef Quaternion<double> Quaterniond;

class PIDController {
public:
    PIDController(float kp, float ki, float kd);
    PIDController(float kp, float ki, float kd, float maxOutput);
    PIDController(float kp, float ki, float kd, float maxOutput, float minOutput);

    void Update(float error, uint64_t dt);

    // Set the accumulated integral value. Units are in error*time.
    void setIntegral(float integralValue);
    
    // Invert the direction of the recieved output for PID controllers
    // Producing positive feedback loops instead of negative ones
    void invertFeedback(bool invert);

    // Current output from the PID controller
    float getOutput();
private:
    // Gains
    float kp;
    float ki;
    float kd;

    // Bounds
    float maxOutput = 1;
    float minOutput = -1;

    // Saved values
    float errorPrior = 0;
    float integralPrior = 0;
    float output = 0;

    // Whether to invert the direction of the output
    bool invertOutput = false;
};


class NonLinearQuaternionController {
public:
    NonLinearQuaternionController();
    NonLinearQuaternionController(float Pq, float Pw);

    void setPq(float Pq) {this->m_Pq = Pq;};
    void setPw(float Pw) {this->m_Pw = Pw;};

    float getPq() {return this->m_Pq;};
    float getPw() {return this->m_Pw;};

    Vector3f Update(Quaternionf qref, Quaternionf qm, Vector3f w);
private:
    float m_Pq, m_Pw;
};

};

#endif // CSMUTIL_HPP