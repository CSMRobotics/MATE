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
    const T* getComponents();
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
    // Vector cpp operators (not going to make version for intercomparibility of different types of Vector3s because i dont want to :) ) 
    bool operator==(const Vector3<T>& rhs);
    bool operator!=(const Vector3<T>& rhs);


/* declare new typename to not step on T
 * and allow cout << Vector3<any>
*/
template<typename Y>
friend std::ostream& operator<<(std::ostream& os, const Vector3<Y>& vector);
private:
    T m_ijk[3] = {0};
};

template<typename T>
std::ostream& operator<<(std::ostream& os, const Vector3<T>& vector);

// heres some handy ones
typedef Vector3<int> Vector3i;
typedef Vector3<float> Vector3f;
typedef Vector3<double> Vector3d;

};

#endif // CSMUTIL_HPP
