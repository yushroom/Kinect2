#ifndef RTMATH_H
#define RTMATH_H
#include "RenderFish.hpp"
#include <iostream>
#include <cassert>
#include <cmath>
#include <cstdint>

#include "MathHelper.hpp"
#include "Vector.hpp"
#include "Matrix4x4.hpp"
//#include "Random.hpp"

// note: Vector in RenderFish is COLUMN vector
// Matrix4x4 in RenderFish is ROW matrix
// matrix-vector multiplication: mat * vec;

//static float dot(const Vec3& u, const Vec3& v)
//{
//	return u.x * v.x + u.y * v.y + u.z * v.z;
//}

//static Vec3 cross(const Vec3& u, const Vec3& v)
//{
//	return Vec3(u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z, u.x * v.y - u.y * v.x);
//}

//#ifndef swap
//#define swap(x, y) {auto t = x; x = y; y = t;}

static Vec3 normalize(const Vec3& v)
{
	//float l = v.length();
	return v.normalize();
}

static Normal normalize(const Normal& n) {
	float l = sqrtf(n.x*n.x + n.y*n.y + n.z*n.z);
	return Normal(n.x / l, n.y / l, n.z / l);
}

static float distance(const Point &p1, const Point &p2)
{ 
	return (p1 - p2).length(); 
}

static float distance_squared(const Point &p1, const Point &p2) 
{
	return (p1 - p2).length_squared();
}

static Point center(const Point& p1, const Point& p2)
{
	return Point((p1.x + p2.x) * 0.5f, (p1.y + p2.y) * 0.5f, (p1.z + p2.z) * 0.5f);
}

// in and normal are normalized
static Vec3 reflect_normalized(const Vec3& in, const Vec3& normal)
{
	//return (n * 2 * (dot(n, v)) - v).normalize();
	return (in - 2 * dot(in, normal) * normal).normalize();
}

static bool perpendicular(const Vec3& u, const Vec3& v)
{
	return zero(dot(u, v));
}

float determinant(const Matrix4x4& rhs);

Matrix4x4 inverse(const Matrix4x4& rhs);

inline Matrix4x4 transpose(const Matrix4x4& m)
{
	return Matrix4x4::transpose(m);
}

// t0 and t1 are roots of Ax^2 + Bx + C = 0
// t0 <= t1
bool quadratic(float A, float B, float C, float *t0, float *t1);

// a00 a01  x0  b0
// a10 a11  x1  b1
bool solve_linear_system_2x2(float a00, float a01, float a10, float a11, float b0, float b1, float* x0, float* x1);

template<class T>
inline void coordinate_system(const T &v1, T *v2, T *v3)
{
	if (fabsf(v1.x) > fabsf(v1.y)) {
		float inv_len = 1.f / sqrtf(v1.x*v1.x + v1.z*v1.z);
		*v2 = T(-v1.z * inv_len, 0.f, v1.x * inv_len);
	}
	else {
		float inv_len = 1.f / sqrtf(v1.y * v1.y + v1.z * v1.z);
		*v2 = T(0.f, v1.z * inv_len, -v1.y * inv_len);
	}
	*v3 = cross(v1, *v2);
}

#endif