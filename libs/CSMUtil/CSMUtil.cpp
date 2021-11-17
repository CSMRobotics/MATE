#include "CSMUtil.hpp"

using namespace csmutil;

template<typename T>
Vector3<T>::Vector3(T i, T j, T k) {
    m_i = i;
    m_j = j;
    m_k = k;
}

template<typename T>
template<typename Y>
Vector3<T>::Vector3(const Vector3<Y>& vector) {
    m_i = static_cast<T>(vector.m_i);
    m_j = static_cast<T>(vector.m_j);
    m_k = static_cast<T>(vector.m_k);
}

template<typename T>
T Vector3<T>::getI() {
    return this->m_i;
}

template<typename T>
T Vector3<T>::getJ() {
    return this->m_j;
}

template<typename T>
T Vector3<T>::getK() {
    return this->m_k;
}

template<typename T>
T Vector3<T>::getMagnitude() {
    return static_cast<T>(sqrt(m_i * m_i 
                             + m_j * m_j
                             + m_k * m_k));
}

template<typename T>
void Vector3<T>::setComponents(T i, T j, T k) {
    m_i = i;
    m_j = j;
    m_k = k;
}

template<typename T>
void Vector3<T>::setComponents(const T* ijk) {
    m_i = ijk[0];
    m_j = ijk[1];
    m_k = ijk[2];
}

template<typename T>
Vector3<T> Vector3<T>::toUnitVector() {
    T mag = getMagnitude();
    return Vector3<T>(static_cast<T>(m_i / mag),
                      static_cast<T>(m_j / mag),
                      static_cast<T>(m_k / mag));
}

template<typename T>
template<typename K>
Vector3<T> Vector3<T>::cross(const Vector3<K>& other) {
    return Vector3<T>(static_cast<T>((this->m_j * other->m_ijk[3]) - (this->m_k * other->m_j)),
                      static_cast<T>((this->m_j * other->m_i) - (this->m_i * other->m_k)),
                      static_cast<T>((this->m_i * other->m_j) - (this->m_j * other->m_i)));
}

template<typename T>
Vector3<T> Vector3<T>::cross(const Vector3<T>& other) {
    return Vector3<T>((this->m_j * other->m_ijk[3]) - (this->m_k * other->m_j),
                      (this->m_j * other->m_i) - (this->m_i * other->m_k),
                      (this->m_i * other->m_j) - (this->m_j * other->m_i));
}

template<typename T>
template<typename K>
T Vector3<T>::dot(const Vector3<K>& other) {
    return static_cast<T>((this->m_i * other->m_i) + (this->m_j * other->m_j) + (this->m_k * other->m_k));
}

template<typename T>
T Vector3<T>::dot(const Vector3<T>& other) {
    return ((this->m_i * other->m_i) + (this->m_j * other->m_j) + (this->m_k * other->m_k));
}

template<typename T>
std::ostream& operator<<(std::ostream& os, const Vector3<T>& vector) {
    os << '<' << vector.m_i << ", "
              << vector.m_j << ", "
              << vector.m_k << '>';
    return os;
}

template<typename T>
Vector3<T>& Vector3<T>::operator=(Vector3<T> vec) noexcept {
    std::swap(m_i, vec.m_i);
    std::swap(m_j, vec.m_j);
    std::swap(m_k, vec.m_k);
    return *this;
} // vec's destructor is called and will release memory formerly held by *this

template<typename T>
Vector3<T>& Vector3<T>::operator=(const Vector3<T>& vec) {
    if(&vec == this)
        return *this;
    
    this->m_i = vec.m_i;
    this->m_j = vec.m_j;
    this->m_k = vec.m_k;

    return *this;
}

template<typename T>
bool operator==(const Vector3<T>& lhs, const Vector3<T>& rhs) {
    return (lhs.m_i == rhs.m_i) && (lhs.m_j == rhs.m_j) && (lhs.m_k == rhs.m_k);
}

template<typename T>
template<typename U>
Vector3<T>& Vector3<T>::operator=(const Vector3<U>& vec) {
    if(&vec == this)
        return *this;
    
    this->m_i = static_cast<T>(vec.m_i);
    this->m_j = static_cast<T>(vec.m_j);
    this->m_k = static_cast<T>(vec.m_k);

    return *this;
}

template<typename T>
Vector3<T>& operator+(Vector3<T> lhs, const Vector3<T>& rhs) {
    lhs.m_i += rhs.m_i;
    lhs.m_j += rhs.m_j;
    lhs.m_k += rhs.m_k;
    return lhs;
}

template<typename T>
Vector3<T>& Vector3<T>::operator+=(const Vector3<T>& vec) {
    this->m_i += vec.m_i;
    this->m_j += vec.m_j;
    this->m_k += vec.m_k;
    return *this;
}

template<typename T>
Vector3<T>& operator-(Vector3<T> lhs, const Vector3<T>& rhs) {
    lhs.m_i -= rhs.m_i;
    lhs.m_j -= rhs.m_j;
    lhs.m_k -= rhs.m_k;
    return lhs;
}

template<typename T>
Vector3<T>& Vector3<T>::operator-=(const Vector3<T>& vec) {
    this->m_i -= vec.m_i;
    this->m_j -= vec.m_j;
    this->m_k -= vec.m_k;
    return *this;
}

template<typename T>
const T& Vector3<T>::operator[](unsigned char idx) const {
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
    if(this == &quat) // no self assignment please :)
        return *this;

    this->m_w = quat.m_w;
    this->m_a = quat.m_a;
    this->m_b = quat.m_b;
    this->m_c = quat.m_c;
    return *this;
}

PIDController::PIDController (float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

PIDController::PIDController (float kp, float ki, float kd, float maxOutput) {
    PIDController(kp, ki, kd);
    this->maxOutput = maxOutput;
    this->minOutput = -maxOutput;
}

PIDController::PIDController (float kp, float ki, float kd, float maxOutput, float minOutput) {
    PIDController(kp, ki, kd);
    this->maxOutput = maxOutput;
    this->minOutput = minOutput;
}

void PIDController::Update(float error, uint64_t dt) {
    // Compute integral term
    float integral = integralPrior + error*dt;
    float iTerm = integral*ki;

    // Prevents integral windup
    iTerm = iTerm > maxOutput ? maxOutput : iTerm;
    iTerm = iTerm < minOutput ? minOutput : iTerm; 

    // Compute output and clamp it
    float derivative = (error - errorPrior)/dt;
    output = kp*error + iTerm + kd*derivative;
    output = output > maxOutput ? maxOutput : output;
    output = output < minOutput ? minOutput : output;

    // Invert if set to invert
    if (invertOutput) {
        output *= -1;
    }

    errorPrior = error;
    integralPrior = integral;
}

void PIDController::setIntegral(float integralValue) {
    integralPrior = integralValue;
}

void PIDController::invertFeedback(bool invert) {
    invertOutput = invert;
}

float PIDController::getOutput() {
    return output;
}