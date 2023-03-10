#ifndef VEC_REC_UTIL_HEADER_INCLUDED
#define VEC_REC_UTIL_HEADER_INCLUDED

#include <optional>
#include <iostream>
#include <raylib.h>

std::ostream& operator<<(std::ostream& os, Vector2 vec){return os << '[' << vec.x << ", " << vec.y << ']';}
std::ostream& operator<<(std::ostream& os, Vector3 vec){return os << '[' << vec.x << ", " << vec.y << ", " << vec.z << ']';}
std::ostream& operator<<(std::ostream& os, Vector4 vec){return os << '[' << vec.x << ", " << vec.y << ", " << vec.z << ", " << vec.w << ']';}
std::ostream& operator<<(std::ostream& os, Rectangle rec){return os << '[' << rec.width << " x " << rec.height << " @ (" << rec.x << ", " << rec.y <<')' << ']';}

Rectangle operator+(Rectangle lhs, Vector2 rhs){return Rectangle{lhs.x + rhs.x, lhs.y + rhs.y, lhs.width, lhs.height};};
Rectangle& operator+=(Rectangle& lhs, Vector2 rhs){lhs.x += rhs.x; lhs.y += rhs.y; return lhs;};
Rectangle operator-(Rectangle lhs, Vector2 rhs){return Rectangle{lhs.x - rhs.x, lhs.y - rhs.y, lhs.width, lhs.height};};
Rectangle& operator-=(Rectangle& lhs, Vector2 rhs){lhs.x -= rhs.x; lhs.y -= rhs.y; return lhs;};

Vector2 operator+(Vector2 lhs, Vector2 rhs){return Vector2{lhs.x + rhs.x, lhs.y + rhs.y};};
Vector2& operator+=(Vector2& lhs, Vector2 rhs){lhs.x += rhs.x; lhs.y += rhs.y; return lhs;};
Vector2 operator-(Vector2 lhs, Vector2 rhs){return Vector2{lhs.x - rhs.x, lhs.y - rhs.y};};
Vector2& operator-=(Vector2& lhs, Vector2 rhs){lhs.x -= rhs.x; lhs.y -= rhs.y; return lhs;};

Vector3 operator+(Vector3 lhs, Vector3 rhs){return Vector3{lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};};
Vector3& operator+=(Vector3& lhs, Vector3 rhs){lhs.x += rhs.x; lhs.y += rhs.y; lhs.z += rhs.z; return lhs;};
Vector3 operator-(Vector3 lhs, Vector3 rhs){return Vector3{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};};
Vector3& operator-=(Vector3& lhs, Vector3 rhs){lhs.x -= rhs.x; lhs.y -= rhs.y; lhs.z -= rhs.z; return lhs;};

Vector4 operator+(Vector4 lhs, Vector4 rhs){return Vector4{lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z, lhs.w + rhs.w};};
Vector4& operator+=(Vector4& lhs, Vector4 rhs){lhs.x += rhs.x; lhs.y += rhs.y; lhs.z += rhs.z; lhs.w += rhs.w; return lhs;};
Vector4 operator-(Vector4 lhs, Vector4 rhs){return Vector4{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z, lhs.w - rhs.w};};
Vector4& operator-=(Vector4& lhs, Vector4 rhs){lhs.x -= rhs.x; lhs.y -= rhs.y; lhs.z -= rhs.z; lhs.w -= rhs.w; return lhs;};

#endif
