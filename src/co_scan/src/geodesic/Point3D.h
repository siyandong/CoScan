// Point3D.h: interface for the CPoint3D class.
//
//////////////////////////////////////////////////////////////////////

#pragma once
#include <math.h>
#define _USE_MATH_DEFINES
#include <ostream>
using namespace std;

struct CPoint3D  
{
public:
	double x, y, z;
	CPoint3D();
	CPoint3D(double x, double y, double z);	
	inline CPoint3D& operator +=(const CPoint3D& pt);
	inline CPoint3D& operator -=(const CPoint3D& pt);
	inline CPoint3D& operator *=(double times);
	inline CPoint3D& operator /=(double times);
	inline CPoint3D Rotate() const;
	inline double Len() const;
	inline void Normalize();
	bool operator==(const CPoint3D& other) const
	{
		return x == other.x && y == other.y && z == other.z;
	}
	bool operator<(const CPoint3D& other) const
	{
		if (x < other.x)
			return true;
		else if (x > other.x)
			return false;
		else if (y < other.y)
			return true;
		else if (y > other.y)
			return false;
		else if (z < other.z)
			return true;
		else if (z > other.z)
			return false;
		return false;
	}
	friend ostream& operator<<(ostream& out, const CPoint3D& pt)
	{
		out << pt.x << " " << pt.y << " " << pt.z << endl;
		return out;
	}
};
CPoint3D CPoint3D::Rotate() const
{
	return CPoint3D(z, x, y);
}

CPoint3D& CPoint3D::operator +=(const CPoint3D& pt)
{
	x += pt.x;
	y += pt.y;
	z += pt.z;
	return *this;
}

CPoint3D& CPoint3D::operator -=(const CPoint3D& pt)
{
	x -= pt.x;
	y -= pt.y;
	z -= pt.z;
	return *this;
}

CPoint3D& CPoint3D::operator *=(double times)
{
	x *= times;
	y *= times;
	z *= times;
	return *this;
}

CPoint3D& CPoint3D::operator /=(double times)
{
	x /= times;
	y /= times;
	z /= times;
	return *this;
}

double CPoint3D::Len() const
{
	return sqrt(x * x + y * y + z * z);
}

void CPoint3D::Normalize()
{
	double len = Len();
	x /= len;
	y /= len;
	z /= len;
}

CPoint3D operator +(const CPoint3D& pt1, const CPoint3D& pt2);
CPoint3D operator -(const CPoint3D& pt1, const CPoint3D& pt2);
CPoint3D operator *(const CPoint3D& pt, double times);
CPoint3D operator /(const CPoint3D& pt, double times);
CPoint3D operator *(double times, const CPoint3D& pt);
CPoint3D operator *(const CPoint3D& pt1, const CPoint3D& pt2);
CPoint3D VectorCross(const CPoint3D& pt1, const CPoint3D& pt2, const CPoint3D& pt3);
double operator ^(const CPoint3D& pt1, const CPoint3D& pt2);
double GetTriangleArea(const CPoint3D& pt1, const CPoint3D& pt2, const CPoint3D& pt3);
double AngleBetween(const CPoint3D& pt1, const CPoint3D& pt2);
double AngleBetween(const CPoint3D& pt1, const CPoint3D& pt2, const CPoint3D& pt3);
void VectorCross(const float* u, const float* v, float * n);
float VectorDot(const float* u, const float* v);
float AngleBetween(const float* u, const float* v);
CPoint3D CombinePointAndNormalTo(const CPoint3D& pt, const CPoint3D& normal);
CPoint3D CombineTwoNormalsTo(const CPoint3D& pt1, double coef1, const CPoint3D& pt2, double coef2);	