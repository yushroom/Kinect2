#pragma once

#include <cstring>
#include "Vector.hpp"

//#include <xmmintrin.h>
//#define SHUFFLE_PARAM(x, y, z, w) ((x) | ((y) << 2) | ((z) << 4) | ((w) << 6))
//#define _mm_replicate_x_ps(v) _mm_shuffle_ps((v), (v), SHUFFLE_PARAM(0, 0, 0, 0))
//#define _mm_replicate_y_ps(v) _mm_shuffle_ps((v), (v), SHUFFLE_PARAM(1, 1, 1, 1))
//#define _mm_replicate_z_ps(v) _mm_shuffle_ps((v), (v), SHUFFLE_PARAM(2, 2, 2, 2))
//#define _mm_replicate_w_ps(v) _mm_shuffle_ps((v), (v), SHUFFLE_PARAM(3, 3, 3, 3))
//#define _mm_madd_ps(a, b, c) _mm_add_ps(_mm_mul_ps((a), (b)), (c))
//
//__declspec(align(16))
struct Matrix4x4
{
	float m[4][4];
	
	Matrix4x4() { 
		//ZeroMemory(this, sizeof(*this));
		memset(this, 0, sizeof(*this));
		m[0][0] = m[1][1] = m[2][2] = m[3][3] = 1;
	}
	Matrix4x4(float mat[4][4])
	{
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				m[i][j] = mat[i][j];
	}
	Matrix4x4(float m11, float m12, float m13, float m14,
		float m21, float m22, float m23, float m24,
		float m31, float m32, float m33, float m34,
		float m41, float m42, float m43, float m44)
	{
		m[0][0] = m11; m[0][1] = m12; m[0][2] = m13; m[0][3] = m14;
		m[1][0] = m21; m[1][1] = m22; m[1][2] = m23; m[1][3] = m24;
		m[2][0] = m31; m[2][1] = m32; m[2][2] = m33; m[2][3] = m34;
		m[3][0] = m41; m[3][1] = m42; m[3][2] = m43; m[3][3] = m44;
	}

	static Matrix4x4 transpose(const Matrix4x4& m) {
		return Matrix4x4(
			m[0][0], m[1][0], m[2][0], m[3][0],
			m[0][1], m[1][1], m[2][1], m[3][1],
			m[0][2], m[1][2], m[2][2], m[3][2],
			m[0][3], m[1][3], m[2][3], m[3][3]);
	}

	const float* operator[](int index) const { return m[index]; }
	float* operator[](int index) { return m[index]; }
	float& operator()(int row, int col) { return m[row][col]; }
	float operator()(int row, int col) const { return m[row][col]; }

	//static const Vec4 mul(const Matrix4x4& lhs, const Vec4& rhs)
	//{
	//	Vec4 ret;
	//	__m128 v = _mm_load_ps(&rhs.v[0]);
	//	__m128 Mrow1 = _mm_load_ps(lhs.m[0]);
	//	__m128 Mrow2 = _mm_load_ps(lhs.m[1]);
	//	__m128 Mrow3 = _mm_load_ps(lhs.m[2]);
	//	__m128 Mrow4 = _mm_load_ps(lhs.m[3]);

	//	__m128 result;
	//	result = _mm_mul_ps(_mm_replicate_x_ps(v), Mrow1);
	//	result = _mm_madd_ps(_mm_replicate_y_ps(v), Mrow2, result);
	//	result = _mm_madd_ps(_mm_replicate_z_ps(v), Mrow3, result);
	//	result = _mm_madd_ps(_mm_replicate_w_ps(v), Mrow4, result);
	//	
	//	_mm_store_ps(&ret.v[0], result);
	//	return ret;
	//}

	//static const Matrix4x4 mul(const Matrix4x4& lhs, const Matrix4x4& rhs)
	//{
	//	Matrix4x4 ret;
	//	__m128 lrow[4], rcol[4];
	//	for (int i = 0; i < 4; i++)
	//	{
	//		lrow[i] = _mm_load_ps(lhs.m[i]);
	//		rcol[i] = _mm_load_ps(rhs.m[i]);
	//	}

	//	for (int i = 0; i < 4; i++)
	//	{
	//		__m128 result = _mm_mul_ps(_mm_replicate_x_ps(lrow[i]), rcol[0]);
	//		for (int j = 1; j < 4; j++)
	//			result = _mm_madd_ps(_mm_replicate_y_ps(lrow[i]), rcol[j], result);
	//		_mm_store_ps(ret.m[i], result);
	//	}
	//	return ret;
	//}

	friend const Matrix4x4 operator*(const Matrix4x4& lhs, const Matrix4x4& rhs)
	{
		Matrix4x4 result;
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				//result[i][j] = 0;
				//for (int k = 0; k < 4; k++)
				//{
				//	result[i][j] += lhs[i][k] * rhs[k][j];
				//}
				result.m[i][j] = lhs.m[i][0] * rhs.m[0][j] + 
					lhs.m[i][1] * rhs.m[1][j] + 
					lhs.m[i][2] * rhs.m[2][j] + 
					lhs.m[i][3] * rhs.m[3][j];	// faster
			}
		}
		return result;
	}

	friend const Vec4 operator*(const Matrix4x4& lhs, const Vec4& rhs)
	{
		Vec4 result;
		for (int i = 0; i < 4; i++)
		{
			result[i] = rhs.x * lhs[i][0] + rhs.y * lhs[i][1] + rhs.z * lhs[i][2] + rhs.w * lhs[i][3];
		}
		//result[0] = rhs.x * lhs[0][0] + rhs.y * lhs[0][1] + rhs.z * lhs[0][2] + rhs.w * lhs[0][3];
		//result[1] = rhs.x * lhs[1][0] + rhs.y * lhs[1][1] + rhs.z * lhs[1][2] + rhs.w * lhs[1][3];
		//result[2] = rhs.x * lhs[2][0] + rhs.y * lhs[2][1] + rhs.z * lhs[2][2] + rhs.w * lhs[2][3];
		//result[3] = rhs.x * lhs[3][0] + rhs.y * lhs[3][1] + rhs.z * lhs[3][2] + rhs.w * lhs[3][3];

		return result;
	}

	friend const Vec3 operator*(const Matrix4x4& lhs, const Vec3& rhs)
	{
		Vec3 result;
		for (int i = 0; i < 3; i++)
		{
			result[i] = rhs.x * lhs.m[i][0] + rhs.y * lhs.m[i][1] + rhs.z * lhs.m[i][2];
		}
		return result;
	}

	void operator*=(const Matrix4x4& rhs) {
		for (int i = 0; i < 4; i++) {
			float f[4];
			for (int j = 0; j < 4; j++) {
				f[j] = m[i][0] * rhs.m[0][j] + m[i][1] * rhs.m[1][j] +
					   m[i][2] * rhs.m[2][j] + m[i][3] * rhs.m[3][j];
			}
			for (int j = 0; j < 4; j++) {
				m[i][j] = f[j];
			}
		}
	}
};

