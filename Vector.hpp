#ifndef VECTOR_H
#define VECTOR_H

#include <cmath>
#include <iostream>

#include "Error.hpp"
#include "MathHelper.hpp"

template<typename T>
class Vector2D {
public:
	T x, y;
	Vector2D() : x(0), y(0) {}
	Vector2D(T x, T y) : x(x), y(y) {}
};

typedef Vector2D<float> Vec2;
typedef Vector2D<int> IntVec2;

//class Vec2 {
//public:
//	float x, y;
//	
//	Vec2(float x, float y) : x(x), y(y) {}
//};

template < class T, class U >
inline float dot(const T& lhs, const U& rhs)
{
	return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

template < class T, class U >
static T cross(const T& u, const U& v)
{
	return T(u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z, u.x * v.y - u.y * v.x);
}

class Normal;
class Point;

class Vec3 {
public:
	union {
		struct { float x, y, z; };
		float m[3];
	};

	static const Vec3 axis_x;
	static const Vec3 axis_y;
	static const Vec3 axis_z;

	Vec3() : Vec3(0, 0, 0) {}
	Vec3(float x) : Vec3(x, x, x) {}
	Vec3(const Vec3& v) : Vec3(v.x, v.y, v.z) {
	}
	Vec3(float x, float y, float z) : x(x), y(y), z(z) { Assert(!has_NaNs()); }
	explicit Vec3(const Normal& n);
	explicit Vec3(const Point& p);

	bool has_NaNs() const { return isnan(x) || isnan(y) || isnan(z); }

	//const Point as_point() const { return Point(x, y, z); }

	float length_squared() { return x*x + y*y + z*z; }
	float length() const { return sqrtf(x*x + y*y + z*z); }
	const Vec3 normalize() const { float l = length(); return Vec3(x / l, y / l, z / l); }
	void normalize_self() { float l = length(); x /= l, y /= l; z /= l; }

	float&		operator[](const int index) { return m[index]; }
	float		operator[](const int index) const { return m[index]; }
	Vec3&		operator=(const Vec3& v) { x = v.x; y = v.y; z = v.z; return *this; }
	Vec3		operator-() const { return Vec3(-x, -y, -z); }
	Vec3		operator*(const float f) const { return Vec3(x * f, y * f, z * f); }
	Vec3		operator/(const float f) const { return Vec3(x / f, y / f, z / f); }
	friend Vec3 operator*(const float f, const Vec3& v) { return Vec3(v.x * f, v.y * f, v.z * f); }
	friend Vec3 operator/(const float f, const Vec3& v) { return Vec3(v.x / f, v.y / f, v.z / f); }
	Vec3		operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
	Vec3		operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
	void		operator+=(const Vec3& v) { x += v.x; y += v.y; z += v.z; }
	void		operator-=(const Vec3& v) { x -= v.x; y -= v.y; z -= v.z; }
	void		operator*=(const float f) { x *= f; y *= f; z *= f; }
	void		operator/=(const float f) { x /= f; y /= f; z /= f; }
	bool		operator==(const Vec3& v) const { return (equal(x, v.x) && equal(y, v.y) && equal(z, v.z)); }
	bool		operator!=(const Vec3& v) const { return !operator==(v); }

	friend std::ostream& operator << (std::ostream& os, const Vec3& v) { os << v.x << ", " << v.y << ", " << v.z; return os; }
};

class Point
{
public:
	union {
		struct {
			float x, y, z;
		};
		float m[3];
	};

	static const Point zero;
	
	Point() : Point(0, 0, 0) {}
	Point(float x, float y, float z) : x(x), y(y), z(z) {}

	Point operator+(const Vec3 &v) const { return Point(x + v.x, y + v.y, z + v.z); }
	Point operator+(const Point &v) const { return Point(x + v.x, y + v.y, z + v.z); }
	Point operator-(const Vec3 &v) const { return Point(x - v.x, y - v.y, z - v.z); }
	Vec3 operator-(const Point &v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
	Point& operator+=(const Vec3 &v) { x += v.x; y += v.y; z += v.z; return *this; }
	Point& operator-=(const Vec3 &v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
	float& operator[](int index) { return m[index]; }
	float  operator[](int index) const { return m[index]; }
	Point operator*(float f) const { return Point(x * f, y * f, z * f); }
	friend Point operator*(float f, const Point& p) { return Point(p.x * f, p.y * f, p.z * f); }


	bool operator==(const Vec3& p) const { return equal(x, p.x) && equal(y, p.y) && equal(z, p.z); }
	bool operator!=(const Vec3& p) const { return !operator==(p); }
};

class Normal
{
public:
	float x, y, z;

	Normal() : Normal(0, 0, 0) {}
	explicit Normal(float x, float y, float z) : x(x), y(y), z(z) {}
	explicit Normal(const Vec3& v) : x(v.x), y(v.y), z(v.z) {}
	//explicit Normal(const Normal& n) : x(n.x), y(n.y), z(n.z) {}

	Normal& operator=(const Normal& v) { x = v.x; y = v.y; z = v.z; return *this; }
	Normal operator-() const { return Normal(-x, -y, -z); }
	Normal operator*(const float f) const { return Normal(x * f, y * f, z * f); }
	Normal operator/(const float f) const { return Normal(x / f, y / f, z / f); }
	friend Normal operator*(const float f, const Normal& v) { return Normal(v.x * f, v.y * f, v.z * f); }
	friend Normal operator/(const float f, const Normal& v) { return Normal(v.x / f, v.y / f, v.z / f); }
	Normal operator+(const Normal& v) const { return Normal(x + v.x, y + v.y, z + v.z); }
	Normal operator-(const Normal& v) const { return Normal(x - v.x, y - v.y, z - v.z); }
	void operator+=(const Normal& v) { x += v.x; y += v.y; z += v.z; }
	void operator-=(const Normal& v) { x -= v.x; y -= v.y; z -= v.z; }
	void operator*=(const float f) { x *= f; y *= f; z *= f; }
	void operator/=(const float f) { x /= f; y /= f; z /= f; }

	static Normal face_forward(const Normal& n, const Vec3& v) { return (dot(n, v) < 0.f) ? -n : n; }
};

//__declspec(align(16))
class Vec4 {
public:
	union {
		struct {
			float x, y, z, w;
		};
		float v[4];
	};

	Vec4() : Vec4(0, 0, 0, 0) {}
	Vec4(float x, float y, float z, float w)
		: x(x), y(y), z(z), w(w) {}
	explicit Vec4(const Vec3& v3, float w = 0)
		: x(v3.x), y(v3.y), z(v3.z), w(w) {
	}
	explicit Vec4(const Point& p, float w = 1)
		: x(p.x), y(p.y), z(p.z), w(w) {}

	float& operator[](int index) { return v[index]; }
	float operator[](int index) const { return v[index]; }

	// REFINE
	Vec3 xyz() const { return Vec3(x, y, z); }
	Point as_point() const{ return Point(x / w, y / w, z / w); }
};

inline Vec3 spherical_direction(float sin_theta, float cos_theta, float phi) {
	return Vec3(sin_theta * cosf(phi), sin_theta * sinf(phi), cos_theta);
}

// x, y, z basis vectors
inline Vec3 spherical_direction(float sin_theta, float cos_theta, float phi,
	const Vec3 &x, const Vec3 &y, const Vec3 &z) {
	return sin_theta * cosf(phi)* x + sin_theta * sinf(phi) * y + cos_theta * z;
}

inline float spherical_theta(const Vec3 &v) {
	return acosf(clamp(v.z, -1.f, 1.f));
}

inline float spherical_phi(const Vec3 &v) {
	float p = atan2f(v.y, v.x);
	return (p < 0.f) ? p + 2.f*M_PI : p;
}

#endif // VECTOR_H