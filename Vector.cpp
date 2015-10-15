#include "Vector.hpp"

const Point Point::zero(0, 0, 0);

const Vec3 Vec3::axis_x(1, 0, 0);
const Vec3 Vec3::axis_y(0, 1, 0);
const Vec3 Vec3::axis_z(0, 0, 1);

Vec3::Vec3(const Normal & n) : x(n.x), y(n.y), z(n.z)
{
}

Vec3::Vec3(const Point & p) : x(p.x), y(p.y), z(p.z)
{
}
