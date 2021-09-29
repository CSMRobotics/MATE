#ifndef VECTOR3F_HPP
#define VECTOR3F_HPP

class Vector3f {
public:
    Vector3f(float i, float j, float k);
    ~Vector3f();

    float getI();
    float getJ();
    float getK();
    float* getComponents();
    float getMagnitude();
    Vector3f getUnitVector();

    void setComponents(float i, float j, float k);
    void setComponents(const float* ijk);

    Vector3f crossProduct(const Vector3f &other);
    float dotProduct(const Vector3f &other);
protected:

private:
    float i = 0.0f;
    float j = 0.0f;
    float k = 0.0f;
};

#endif // VECTOR3F_HPP